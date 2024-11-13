import sys
import yaml
import cv2
import numpy as np
from skimage import morphology
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

##############################################################################################################################
# Extraccion de mapa topologico basado en mapa geometrico
# Para lanzarlo, utilizar el comando: python topoMap.py nombreDelMapa.yaml
# - Entrada: archivo .yaml correspondiente al mapa geometrico
# - Salida: - archivo mapaTopo.txt con la informacion de los nodos correspondientes a las zonas de conexion [X Y zona1 zona2]
#           - archivo subNodos.txt con la informacion de los sub-nodos dentro de cada zona [X Y zona]
##############################################################################################################################

def segWatershed(mapa,mapaBinario):
    # Aplicacion del metodo Watershed: 
    
    #1) Para este proceso se debe realizar un preprocesado del mapa mediante una dilatacion y una transformacion de distancia usando OpenCV.

	# Dilatar el mapa para unir zonas cercanas
	kernel = np.ones((5,5),np.uint8)
	fondo= cv2.dilate(mapaBinario, kernel, iterations = 2)

	# Applicar la transformada de la distancia para separar las zonas
	dist_transform = cv2.distanceTransform(fondo, cv2.DIST_L2, 5)


	#2)Modificar el umbral (0.2) a aplicar en la transformada de la distanca y observar el efecto en la segmentacion final
	_, frente = cv2.threshold(dist_transform,0.5*dist_transform.max(),255,0)

	desconocido = cv2.subtract(fondo,np.uint8(frente))
	_, etiquetas = cv2.connectedComponents(np.uint8(frente))
	etiquetas = etiquetas+1
	etiquetas[desconocido==255] = 0

	mapa = cv2.cvtColor(mapa, cv2.COLOR_GRAY2BGR)
	etiquetas = cv2.watershed(mapa,etiquetas)

	#Modificamos la matriz para visualizarla con color
	colormap = np.random.randint(0, 255, (etiquetas.max() + 1, 3), dtype=np.uint8)
	colormap[1] = [255, 255, 255]
	etiquetas_coloreadas = colormap[etiquetas]
	etiquetas_coloreadas = cv2.convertScaleAbs(etiquetas_coloreadas)

	return etiquetas, etiquetas_coloreadas

def extraerMapaTopo(etiquetas):
	print("Tipo de dato etiqeuta:",type(mapa))

	#1) Extraemos los perimetros de cada region segmentada
	# Pista: tener en cuenta que la etiqueta '1' corresponde a los pixeles del fondo, empezar el bucle en 2. Usar findContours de cv2.
	perimetros=[]
	for i in range(2, etiquetas.max() + 1):
		# Find contours for the current label
		contours, _ = cv2.findContours((etiquetas == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		# Store the x and y coordinates for each contour
		for contour in contours:
			perimetros.append(contour.squeeze())

	# Print the perimeters with their respective indices
	for index, perimetro in enumerate(perimetros):
		print(f"Perimeter {index+1}: {perimetro}")

	#2) Select the points that divide two regions (values -1 between two positive values different from 1)
	vecinos=[]
	pxVecinos=[]
	indices = np.where(etiquetas == -1) #We save the indices of the edges
	indices_as_tuples = np.transpose(indices)

	#Hint: Use a for loop that goes through all the perimeters. For each pixel, its 8 neighbors will be saved. Of all of them, we only save those that divide two areas that are not background.
	# pxVecinos saves the i,j coordinates of the pixels where there is separation between rooms.
	# vecinos saves the value of the label of the two rooms that coincide in the corresponding pixel of pxVecinos
	
	for i, j in indices_as_tuples:
		# Get the 8 neighbors for the current pixel
		neighbors = etiquetas[max(i-1, 0):min(i+2, etiquetas.shape[0]), max(j-1, 0):min(j+2, etiquetas.shape[1])]
		# Get the unique labels of the neighbors
		unique_labels = np.unique(neighbors)
		# If there are more than 2 unique labels and none of them is 1 (background)
		if len(unique_labels) > 2 and not np.isin(1, unique_labels):
			# Save the coordinates of the current pixel
			pxVecinos.append((i, j))
			# Save the labels of the neighbors
			vecinos.append(unique_labels[unique_labels != -1])

	# Guardamos los puntos centrales de cada separacion y sus conexiones
	separaciones = np.zeros((etiquetas.shape[0], etiquetas.shape[1]), dtype=np.uint8)
	for pixel in pxVecinos:
		separaciones[pixel[0], pixel[1]] = 255
	num_labels, labeled_image, stats, centroids  = cv2.connectedComponentsWithStats(separaciones, 8) #Analizamos cada separacion por separado
	puntosCentrales=[]
	conexiones=[]
	for label in range(1, num_labels):
		pixels = np.where(labeled_image == label)
		pxCentral = (pixels[0][int(len(pixels[0])/2)], pixels[1][int(len(pixels[0])/2)]) #Punto central de cada separacion
		puntosCentrales.append(pxCentral)
		conexionCentral = vecinos[next((i for i, px in enumerate(pxVecinos) if (px == np.array(pxCentral)).all()), None)] #Zonas que coinciden en la separacion
		conexiones.append(conexionCentral)

	# Imprimimos los puntos centrales y sus conexiones
	for index, punto in enumerate(puntosCentrales):
		print(f"Punto Central {index+1}: {punto}")
		print(f"Conexión: {conexiones[index]}")

	return perimetros, puntosCentrales, conexiones

def extraerSubNodos(mapaBinario, perimetros):
	#1) Extraemos puntos relevantes en funcion del esqueleto de la imagen
	#Pista: se debe preprocesar el mapa erosionando para que los subnodos a los que el robot debe dirigirse no sea proximos a las paredes. 
	#Usar medial_axis (libreria morphology) para extraer el esqueleto
	esqueleto = morphology.medial_axis(mapaBinario)

	#Seleccionar px del esqueleto que tenga 1 vecino (esquina) o 3 vecinos (nodo interno)
	coordSubNodos=[]
	esqueletoPx= np.where(esqueleto == 1)
	px_as_tuples = np.transpose(esqueletoPx)
	for px in px_as_tuples:
		idxVecino = np.indices((3, 3)) + np.array(px)[:, np.newaxis, np.newaxis] - 1
		idxVecino = idxVecino.reshape(2, -1).T
		neighbors = esqueleto[tuple(idxVecino.T)] #Array de 9 valores incluyendo el punto seleccionado junto con sus 8 vecinos
		if sum(neighbors)==2 or sum(neighbors)>3: #Guardamos puntos con un vecino o mas de dos
			coordSubNodos.append(px)

	#2) Asignar cada nodo del esqueleto a un perimetro
	#Pista: Hacer un bucle que recorra los perimetros de las hbaitaciones y que compruebe si el subnodo esta dentro de ese perimetro. 
	# Hay que crear un objeto Polygon para cada perímetro, y usar la funcion contains para comrpoabr que el punto esta dentro del perimetro  
	regionSubNodos=[]
	for node in coordSubNodos:
		point = Point((node[1],node[0]))
		for i, perimetro in enumerate(perimetros):
			polygon = Polygon(perimetro)
			if polygon.contains(point):
				regionSubNodos.append(i)
				break

	return esqueleto, coordSubNodos, regionSubNodos

if __name__ == "__main__":

	##############################################################################################################################
	#Paso 1: Cargamos los metadatos del mapa y lo filtramos
	fichero = sys.argv
	with open(fichero[1], 'r') as file:
		metadatos = yaml.safe_load(file)
	matriz = metadatos["image"]
	res = metadatos["resolution"]
	oriX = metadatos["origin"][0]
	oriY = metadatos["origin"][1]

	#Carga la matriz del mapa y filtra con un opening
	mapa = cv2.imread(matriz,-1)
	mapaBinario = np.where((mapa == 254), 255, 0).astype(np.uint8) #Seleccionamos las zonas libres unicamente
	mapaBinario = cv2.morphologyEx(mapaBinario, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
	mapaBinario = cv2.erode(mapaBinario,np.ones((5,5),np.uint8),iterations = 1)

	##############################################################################################################################
	#Paso 2: Calculamos el mapa topologico del entorno
	# Aplicamos Watershed
	# 'etiquetas' es una matriz donde -1 indica borde entre las zonas, 1 es el fondo y el resto de valores (2, 3, etc.) es cada region etiquetada
	# 'etiquetas_coloreadas' es la misma matriz con color para su visualizacion
	
	etiquetas, etiquetas_coloreadas = segWatershed(mapa,mapaBinario)

	#Extraemos informacion relevante: regiones separadas (su perimetro) y puntos de conexion (su punto central y que regiones conectan)
	perimetros, puntosCentrales, conexiones = extraerMapaTopo(etiquetas)

	##############################################################################################################################
	#Paso 3: Extraemos sub-nodos basados en la disposicion de los obstaculos usando diagramas Voronoi
	esqueleto, coordSubNodos, regionSubNodos = extraerSubNodos(mapaBinario, perimetros)

	##############################################################################################################################
	#Paso 4: Cambiamos las coordenadas de pixeles a valores metricos y guardamos la info relevante en un archivo de texto

	#TODO: Cambiar las coordenadas de pixeles a valores metricos usando los metadatos del mapa

	#TODO: Completar la escritura de los erchivos con la informacion relevante
	# with open('mapaTopo.txt', 'w') as file:
	# 	for i in range():
	# 		file.write(f"{}\n")
	# with open('subNodos.txt', 'w') as file:
	# 	for i in range():
	# 		file.write(f"{}\n")

	##############################################################################################################################
	#Visualizamos datos
	overlay = cv2.addWeighted(cv2.cvtColor(mapa, cv2.COLOR_GRAY2BGR), 0.5, etiquetas_coloreadas, 0.5, 0)
	overlay[esqueleto==1]=[0, 0, 0]
	for nodo in coordSubNodos:
		overlay = cv2.circle(overlay, (nodo[1],nodo[0]), 3, (0, 255, 0), 1)
	cv2.imshow('Mapa', overlay)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
