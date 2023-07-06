import mysql.connector
from dotenv import load_dotenv
import os

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

        # total_time = 1
        # load_data_time = 1
        # tsp_time = 1
        # rrt_time = 1
        # folder = "nombre_carpeta"
        # vehicle_capacities = int(os.getenv('VEHICLE_CAPACITIES'))
        # num_rows = int(os.getenv('NUM_ROWS'))
        # expand_distance = os.getenv('EXPAND_DISTANCE')
        # goal_sample_rate = os.getenv('EXPAND_DISTANCE')
        # max_iter = int(os.getenv('MAX_ITER'))
        # radius = os.getenv('RADIUS')
        # method = os.getenv('METHOD')
        # path_length = 10
        total_time = data['total_time']
        load_data_time = data['load_data_time']
        tsp_time = data['tsp_time']
        rrt_time = data['rrt_time']
        folder = data['folder']
        vehicle_capacities = int(os.getenv('VEHICLE_CAPACITIES'))
        num_rows = int(os.getenv('NUM_ROWS'))
        expand_distance = float(os.getenv('EXPAND_DISTANCE'))
        goal_sample_rate = float(os.getenv('EXPAND_DISTANCE'))
        max_iter = int(os.getenv('MAX_ITER'))
        radius = float(os.getenv('RADIUS'))
        method = os.getenv('METHOD')
        path_length = 10

        query = "INSERT INTO results (total_time, load_data_time, tsp_time, rrt_time, folder, vehicle_capacities, num_rows, expand_distance,goal_sample_rate, max_iter, radius, method,path_length) VALUES (%s, %s, %s, %s,%s, %s, %s, %s,%s, %s, %s, %s, %s)"
        values = (
            total_time, load_data_time, tsp_time, rrt_time, folder, vehicle_capacities, num_rows, expand_distance,
            goal_sample_rate, max_iter, radius, method, path_length)
        cursor.execute(query, values)
        cnx.commit()
        cursor.close()
        cnx.close()
        print("Data saved successfully")

    except Exception as e:
        print(e)

# save_data_base()
