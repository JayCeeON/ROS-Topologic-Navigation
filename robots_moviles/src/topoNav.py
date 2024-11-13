#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from random import randint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import networkx as nx
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import PoseStamped
import tf


def pose_callback(pose_msg):
    global pose_data
    pose_data = pose_msg

if __name__ == '__main__':
    global pose_data
    try:

        ###### INICIALIZACION ######
        rospy.init_node('topoNav', anonymous=True) #Inicializamos el nodo

        pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback) #Nos suscribimos al topic de la pose
        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction) #Inicializamos el cliente de move_base
        goal_client.wait_for_server() #Esperamos a que el servidor de move_base este activo
        goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10) #Inicializamos el publisher del goal
        path_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)#Inicializamos el publisher del path

        rate = rospy.Rate(1) 
        bridge = CvBridge()


        ###### OBTENCIÓN DE LOS NODOS A UTILIZAR DEL TXT GENERADO ######

        #Cargamos archivos con nodos y subnodos del archivo generado
        x_coords = []
        y_coords = []
        regions = []
        with open('/home/jocol/catkin_ws/src/robots_moviles/src/es1_subNodos.txt', 'r') as file:
            for line in file:
                values = line.strip().split(',')
                x_coords.append(float(values[0]))
                y_coords.append(float(values[1]))
                regions.append(values[2].strip())

        # Creaar una lista de los nodos del grafo con sus coordenadas y orientaciones
        nodes = []
        for i in range(len(x_coords)):

            # Creamos un diccionario con la información del nodo
            node = {
                'index': i,
                'x': x_coords[i],
                'y': y_coords[i],
                'orientation': 1
            }
            nodes.append(node)

        # Vamos a eliminar nodos redundantes que estén muy cerca unos de otros:
        non_redundant_nodes = []
        new_index = 0
        for node in nodes:
            # Miramos si el nodo está demasiado cerca de algún otro nodo
            too_close = False
            for nr_node in non_redundant_nodes:
                dist = math.sqrt((node['x'] - nr_node['x'])**2 + (node['y'] - nr_node['y'])**2)
                if dist <= 0.2:
                    too_close = True
                    break
            # Si no está demasiado cerca, lo añadimos a la lista de nodos no redundantes, dándoles un nuevo índice
            if not too_close:
                node['index'] = new_index
                non_redundant_nodes.append(node)
                new_index += 1

        # Actualizamos la lista de nodos, dejando los nodos no redundantes re-indexados
        nodes = non_redundant_nodes #Estos son nuestros nodos finales, que utilizaremos para movernos

        #Visualizar los nodos en RVIZ como marcadores:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        for node in nodes:
            point = Point()
            point.x = node['x']
            point.y = node['y']
            point.z = 0.0
            marker.points.append(point)



        ###### CREAR EL GRAFO SOBRE EL QUE PLANIFICAR ######

        # Vamos a crear un grafo con los nodos y las conexiones entre ellos utilizando la librería NetworkX
        G = nx.Graph()

        #Añadir los nodos al grafo
        for node in nodes:
            G.add_node(node['index'], x=node['x'], y=node['y'], orientation=node['orientation'])

        # Crear las conexiones entre los nodos, no he encontrado una forma de hacerlo automáticamente y que quede bien, así que lo hago manualmente
        connections = [(0,6),(1,6),(6,8),(8,7),(8,11),(11,14),(11,15),(7,2),(7,13),(6,7),(11,7),(13,16),(13,17),(7,10),(10,9),(10,12),(9,12),(10,5),(9,5),(5,3),(5,4),(12,18),(12,19)]  # Add your connections here

        # Añadir las conexiones al grafo
        for connection in connections:
            i, j = connection
            dist = math.sqrt((nodes[i]['x'] - nodes[j]['x'])**2 + (nodes[i]['y'] - nodes[j]['y'])**2)
            G.add_edge(i, j, weight=dist)

        # Ver el grafo
        pos = {node:(G.nodes[node]['x'], G.nodes[node]['y']) for node in G.nodes()}
        nx.draw(G, pos, with_labels=True)
        plt.show()

        while not rospy.is_shutdown():

            # Publish the marker
            marker_pub.publish(marker)
            # Pedimos al usuario que elija un nodo de inicio y un nodo de destino
            start_node_index = int(input("Por favor, elija el índice del nodo de inicio: "))
            end_node_index = int(input("Por favor, elija el índice del nodo de destino: "))


            # Comprobamos que los índices proporcionados son válidos
            if start_node_index < 0 or start_node_index >= len(nodes) or end_node_index < 0 or end_node_index >= len(nodes):
                print("Índice de nodo inválido. Por favor, elija un índice entre 0 y ", len(nodes)-1)
            else:
                print('Destino elegido!')

                # Use Dijkstra's algorithm to find the shortest path
                path = nx.dijkstra_path(G, start_node_index, end_node_index)
                marker_array = MarkerArray()

                # Move the robot to each node in the path
                for nodeIndex in nodes:
                    # Check if the start node index is less than the end node index
                    if start_node_index < end_node_index:
                        orientation_rad = math.pi
                    else:
                        orientation_rad = 1

                # Convert the orientation to a quaternion
                quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation_rad)

                # Move the robot to each node in the path
                for i, nodeIndex in enumerate(path):
                    # Create a marker for the node
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.pose.position.x = nodes[nodeIndex]['x']
                    marker.pose.position.y = nodes[nodeIndex]['y']
                    marker.pose.position.z = 0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.id = i
                    marker.pose.orientation.w = 1.0
                    # Add the marker to the MarkerArray
                    marker_array.markers.append(marker)

                # Publish the MarkerArray
                path_pub.publish(marker_array)
                marker_pub.publish(marker)

                # Move the robot to each node in the path
                for nodeIndex in path:
                    # Convert the orientation from degrees to radians
                    orientation_rad = math.radians(nodes[nodeIndex]['orientation'])

                    # Convert the orientation to a quaternion
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation_rad)

                    # Crear un mensaje de tipo PoseStamped para publicar el destino y verlo en rviz
                    goal_msg = PoseStamped()
                    goal_msg.header.frame_id = "map"
                    goal_msg.header.stamp = rospy.Time.now()
                    goal_msg.pose.position.x = nodes[nodeIndex]['x']
                    goal_msg.pose.position.y = nodes[nodeIndex]['y']
                    # Set the orientation of the goal
                    goal_msg.pose.orientation.x = quaternion[0]
                    goal_msg.pose.orientation.y = quaternion[1]
                    goal_msg.pose.orientation.z = quaternion[2]
                    goal_msg.pose.orientation.w = quaternion[3]

                    # Publicar el destino para su visualización en rviz
                    goal_pub.publish(goal_msg)

                    #Enviamos la coordenada a move_base y esperamos a llegar
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = nodes[nodeIndex]['x']
                    goal.target_pose.pose.position.y = nodes[nodeIndex]['y']
                    # Set the orientation of the goal
                    goal.target_pose.pose.orientation.x = quaternion[0]
                    goal.target_pose.pose.orientation.y = quaternion[1]
                    goal.target_pose.pose.orientation.z = quaternion[2]
                    goal.target_pose.pose.orientation.w = quaternion[3]

                    goal_client.send_goal(goal)
                    wait = goal_client.wait_for_result()

                    rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
