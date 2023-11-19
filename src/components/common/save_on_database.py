from datetime import datetime

import mysql.connector
import traceback
from colorama import init, Fore
from src.steps import config

init()


def save_data_base(data):
    try:
        cnx = mysql.connector.connect(
            user='root',
            password='12345',
            host='localhost',
            database='tesis'
        )

        cursor = cnx.cursor()

        total_time = data['total_time']
        load_data_time = data['load_data_time']
        tsp_time = data['tsp_time']
        rrt_time = data['rrt_time']
        folder = data['folder']
        vehicle_capacities = int(config.VEHICLE_CAPACITIES)
        num_rows = int(config.NUM_ROWS)
        expand_distance = 0
        goal_sample_rate = float(config.GOAL_SAMPLE_RATE)
        min_inter = int(config.MIN_ITER)
        max_iter = int(config.MAX_ITER)
        radius = float(config.RADIUS)
        method = config.METHOD
        path_length = float(data['path_length_rrt']) / (int(config.RESOLUTION_X) / int(config.MAP_WIDTH))
        total_loaded_tsp = data['total_loaded_tsp']
        total_length_tsp = float(data['total_length_tsp']) / 100
        on_line = config.ON_LINE

        query = "INSERT INTO results (total_time, load_data_time, tsp_time, rrt_time, folder, vehicle_capacities, num_rows, expand_distance,goal_sample_rate, max_iter, radius, method,path_length,min_inter,total_loaded_tsp,total_length_tsp,on_line) VALUES (%s, %s, %s, %s,%s, %s, %s, %s,%s, %s, %s, %s, %s, %s, %s, %s, %s)"
        values = (
            total_time, load_data_time, tsp_time, rrt_time, folder, vehicle_capacities, num_rows, expand_distance,
            goal_sample_rate, max_iter, radius, method, path_length, min_inter, total_loaded_tsp, total_length_tsp,
            on_line)
        cursor.execute(query, values)
        cnx.commit()
        cursor.close()
        cnx.close()
        print(Fore.LIGHTGREEN_EX + "Data saved successfully")

    except Exception as e:
        print(e)
        traceback.print_exc()


def save_data_rrt_test(data):
    try:
        cnx = mysql.connector.connect(
            user='root',
            password='12345',
            host='localhost',
            database='tesis'
        )
        cursor = cnx.cursor()
        method = data['method']
        test_number = data['test_number']
        total_cost = data['total_cost']
        total_collisions = data['total_collisions']
        total_planning_time = data['total_planning_time']
        total_samples = data['total_samples']
        waypoints_number = data['waypoints_number']
        min_iter = config.MIN_ITER
        max_iter = config.MAX_ITER
        date = datetime.now()
        name_folder = data['name_folder']
        success = data['success']
        total_nodes = data['total_nodes']
        extend_dis = config.RADIUS
        neighbor_size = config.NEIGHBOR_SIZE
        time_limit = config.TIME_LIMIT
        query = "INSERT INTO results_tests_rrt (method, test_number, total_cost, total_collisions, total_planning_time, total_samples,waypoints_number, min_iter, max_iter, date, name_folder, success, total_nodes, extend_dis, neighbor_size, time_limit) VALUES (%s, %s, %s, %s,%s, %s,%s, %s, %s,%s,%s, %s, %s,%s, %s, %s)"
        values = (
            method, test_number, total_cost, total_collisions, total_planning_time, total_samples, waypoints_number,
            min_iter, max_iter, date, name_folder, success, total_nodes, extend_dis, neighbor_size, time_limit)
        cursor.execute(query, values)
        cnx.commit()
        cursor.close()
        cnx.close()
    except Exception as e:
        print(e)
        traceback.print_exc()

# save_data_base()
# CREATE TABLE results (
#     id INT AUTO_INCREMENT PRIMARY KEY,
#     total_time INT,
#     load_data_time INT,
#     tsp_time INT,
#     rrt_time INT,
#     folder VARCHAR(255),
#     vehicle_capacities INT,
# 	num_rows INT,
# 	expand_distance VARCHAR(255) NULL,
# 	goal_sample_rate VARCHAR(255) NULL,
# 	max_iter INT,
#     min_iter INT,
# 	radius VARCHAR(255) NULL,
# 	method VARCHAR(255),
# 	path_length VARCHAR(255) NULL,
#     total_loaded_tsp VARCHAR(45) NULL,
#     total_length_tsp VARCHAR(45) NULL
# );
