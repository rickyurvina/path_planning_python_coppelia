import mysql.connector
from dotenv import load_dotenv
import os
import traceback
from colorama import init, Fore

init()
load_dotenv()


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
        vehicle_capacities = int(os.getenv('VEHICLE_CAPACITIES'))
        num_rows = int(os.getenv('NUM_ROWS'))
        expand_distance = 0
        goal_sample_rate = float(os.getenv('GOAL_SAMPLE_RATE'))
        min_inter = int(os.getenv('MIN_ITER'))
        max_iter = int(os.getenv('MAX_ITER'))
        radius = float(os.getenv('RADIUS'))
        method = os.getenv('METHOD')
        path_length = float(data['path_length_rrt']) / (int(os.getenv("RESOLUTION_X")) / int(os.getenv("MAP_WIDTH")))
        total_loaded_tsp = data['total_loaded_tsp']
        total_length_tsp = float(data['total_length_tsp']) / 100
        on_line = os.getenv('ON_LINE')

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
