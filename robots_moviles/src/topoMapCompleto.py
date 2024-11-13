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
    #Aplica Watershed
	fondo = cv2.dilate(mapaBinario,np.ones((3,3),np.uint8),iterations=5)
	dist_transform = cv2.distanceTransform(mapaBinario,cv2.DIST_L2,5)

	#TODO: Modificar el umbral (0.6) a aplicar en la transformada de la distanca y observar el efecto en la segmentacion final
	_, frente = cv2.threshold(dist_transform,0.3*dist_transform.max(),255,0)

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
	#Extraemos los perimetros de cada region segmentada
	perimetros=[]
	for i in range(2, etiquetas.max() + 1):
		mascara = np.where(etiquetas == i, 255, 0).astype(np.uint8)
		perimetro, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		perimetros.append(perimetro)

	#Seleccionamos los puntos que dividen dos regiones (valores -1 entre dos valores positivos diferentes de 1)
	vecinos=[]
	pxVecinos=[]
	indices = np.where(etiquetas == -1) #Guardamos los indices de los bordes
	indices_as_tuples = np.transpose(indices)
	for index in indices_as_tuples:
		if np.any(index[0]!=0) and np.any(index[1]!=0) and np.any(index[0]!=(etiquetas.shape[0]-1)) and np.any(index[1]!=(etiquetas.shape[1]-1)): #Evitamos los bordes
			idxVecino = np.indices((3, 3)) + np.array(index)[:, np.newaxis, np.newaxis] - 1
			idxVecino = idxVecino.reshape(2, -1).T
			neighbors = etiquetas[tuple(idxVecino.T)] #Array de 9 valores incluyendo el punto seleccionado junto con sus 8 vecinos
			if not np.any(neighbors == 1): #Guardamos solo los que dividen dos zonas que no sean fondo
				vecino = np.unique(neighbors[neighbors > 0]) #'index' es un punto seleccionado que divide dos zonas diferentes a 1, y 'vecino' indica que dos regiones se conectan ahi
				vecinos.append(vecino)
				pxVecinos.append(index)

	#Guardamos los puntos centrales de cada separacion y sus conexiones
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

	return perimetros, puntosCentrales, conexiones

def extraerSubNodos(mapaBinario, perimetros):
	#Extraemos puntos relevantes en funcion del esqueleto de la imagen
	mapaErosionado = cv2.morphologyEx(mapaBinario, cv2.MORPH_ERODE, np.ones((15,15),np.uint8)) #Erosionamos para que el robot no se acerque mucho a las paredes
	esqueleto = morphology.medial_axis(mapaErosionado)

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

	#Asignar cada nodo del esqueleto a un perimetro
	regionSubNodos=[]
	for node in coordSubNodos:
		point = Point((node[1],node[0]))
		for i, perimetro in enumerate(perimetros):
			polygon = Polygon(np.squeeze(perimetro))
			if polygon.contains(point): #Comprobamos que el punto este dentro del perimetro
				regionSubNodos.append(i+2) #Sumamos dos porque el perimetro 0 es la zona 2, el perimetro 1 es la zona 3, etc.
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
	print("Puntos centrales:", puntosCentrales)

	##############################################################################################################################
	#Paso 3: Extraemos sub-nodos basados en la disposicion de los obstaculos usando diagramas Voronoi
	esqueleto, coordSubNodos, regionSubNodos = extraerSubNodos(mapaBinario, perimetros)

	##############################################################################################################################
	#Paso 4: Cambiamos las coordenadas de pixeles a valores metricos y guardamos la info relevante en un archivo de texto
	height, width = mapa.shape
	print(conexiones)
	with open('mapaTopo.txt', 'w') as file:
		for i in range(len(puntosCentrales)):
			file.write(f"{(puntosCentrales[i][1]*res) + oriX}, {((height - puntosCentrales[i][0])*res) + oriY}, {conexiones[i][0]}, {conexiones[i][1]}\n")
	with open('subNodos.txt', 'w') as file:
		for i in range(len(coordSubNodos)):
			file.write(f"{(coordSubNodos[i][1]*res) + oriX}, {((height - coordSubNodos[i][0])*res) + oriY}, {regionSubNodos[i]}\n")

	##############################################################################################################################
	#Visualizamos datos
	overlay = cv2.addWeighted(cv2.cvtColor(mapa, cv2.COLOR_GRAY2BGR), 0.5, etiquetas_coloreadas, 0.5, 0)
	overlay[esqueleto==1]=[0, 0, 0]
	for nodo in coordSubNodos:
		overlay = cv2.circle(overlay, (nodo[1],nodo[0]), 3, (0, 255, 0), 1)
	cv2.imshow('Mapa', overlay)
	cv2.waitKey(0)
	cv2.destroyAllWindows()