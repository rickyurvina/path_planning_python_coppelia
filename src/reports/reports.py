from datetime import datetime
import mysql.connector
import traceback
from colorama import init, Fore
from src.components import create_folder
from src.components.common.save_files import get_name_to_save_plot
from src.steps import config
import matplotlib.pyplot as plt
import numpy as np
from prettytable import PrettyTable
import seaborn as sns
import pandas as pd

init()


def average_test(test_number='20231112-d66cad06-6c3a-47e0-8234-99c2c428936a', name_folder='solution_825_12112023_1',
                 path_solutions='../../solutions', num_tests=1):
    conn = mysql.connector.connect(
        user='root',
        password='12345678',
        host='localhost',
        database='tesis'
    )

    cursor = conn.cursor()

    query = """
        SELECT
            method,
            COUNT(*) as total_tests,
            AVG(total_cost) as avg_total_cost,
            AVG(total_collisions) as avg_total_collisions,
            AVG(total_planning_time) as avg_total_planning_time,
            AVG(waypoints_number) as avg_waypoints_number,
            AVG(min_iter) as avg_min_iter,
            AVG(max_iter) as avg_max_iter,
            AVG(total_nodes) as avg_total_nodes,
            AVG(total_samples) as avg_total_samples
        FROM
            results_tests_rrt
        WHERE
            success = true AND
            test_number = %s
        GROUP BY
            method;
    """
    cursor.execute(query, (test_number,))
    # cursor.execute(query, (test_number,))

    results = cursor.fetchall()
    plot_test_rrt(results, name_folder, path_solutions, num_tests)

    for row in results:
        print(row)

    cursor.close()
    conn.close()


def success_failure_rate_by_method(test_number='20231112-d66cad06-6c3a-47e0-8234-99c2c428936a',
                                   name_folder='solution_825_12112023_1',
                                   path_solutions='../../solutions', num_tests=1):
    conn = mysql.connector.connect(
        user='root',
        password='12345678',
        host='localhost',
        database='tesis'
    )

    cursor = conn.cursor()

    query = """
        SELECT
            method,
            success,
            COUNT(*) as total_tests
        FROM
            results_tests_rrt
        WHERE
            test_number = %s
        GROUP BY
            method, success;
    """
    cursor.execute(query, (test_number,))
    # cursor.execute(query)

    results = cursor.fetchall()

    method_success_failure = {}

    for row in results:
        method = row[0]
        success = row[1]
        total_tests = row[2]

        if method not in method_success_failure:
            method_success_failure[method] = {'success': 0, 'failure': 0}

        if success == 1:  # Success
            method_success_failure[method]['success'] += total_tests
        elif success == 0:  # Failure
            method_success_failure[method]['failure'] += total_tests

    cursor.close()
    conn.close()

    for method, rates in method_success_failure.items():
        total_tests_method = rates['success'] + rates['failure']
        success_rate = (rates['success'] / total_tests_method) * 100
        failure_rate = (rates['failure'] / total_tests_method) * 100

        print(f"Method: {method}")
        print(f"  Success Rate: {success_rate}%")
        print(f"  Failure Rate: {failure_rate}%")
        print("---------------------")

    plot_success_failure_bar_by_method(method_success_failure, name_folder, path_solutions, num_tests)


def plot_success_failure_bar_by_method(method_success_failure, name_folder, path_solutions='../../solutions',
                                       num_tests=1):
    methods = list(method_success_failure.keys())
    success_rates = [(rates['success'] / (rates['success'] + rates['failure'])) * 100 for rates in
                     method_success_failure.values()]
    failure_rates = 100 - np.array(success_rates)

    fig, ax = plt.subplots(1, figsize=(12, 10))

    bar_width = 0.35
    index = np.arange(len(methods))

    bars1 = ax.bar(index, success_rates, bar_width, label='Success', color='green')
    ax.bar_label(bars1, fmt='{:,.0f}' + '%', fontsize=20)

    bars2 = ax.bar(index + bar_width, failure_rates, bar_width, label='Failure', color='red')
    ax.bar_label(bars2, fmt='{:,.0f}' + '%', fontsize=20)

    ax.set_xlabel('Method', fontsize="18")
    ax.set_ylabel('Percentage', fontsize="18")
    # ax.set_title('Success-Failure Rate by Method')
    ax.set_xticks(index + bar_width / 2)
    ax.set_xticklabels(methods, fontsize="18")
    yticks = np.arange(0, 101, 20)
    ax.set_yticks(yticks)
    ax.set_yticklabels([f'{val}%' for val in yticks], fontsize="18")

    # Ajustar la posición de la leyenda
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), fontsize=20, fancybox=True, shadow=True, ncol=2)
    ax.set_ylim(0, 100)

    name = 'success_failure_rate_by_method'
    filename = get_name_to_save_plot(name_folder, name, path_solutions, '.svg')
    plt.savefig(filename, format='svg', dpi=500)
    plt.show()


def plot_test_rrt(results, name_folder, path_solutions='../../solutions', num_tests=10):
    methods = [row[0] for row in results]
    num_tests = results[0][1]
    avg_costs = [row[2] for row in results]
    avg_collisions = [row[3] for row in results]
    avg_planning_time = [row[4] for row in results]
    avg_nodes = [row[8] for row in results]
    avg_samples = [row[9] for row in results]

    bar_width = 0.2
    index = np.arange(len(methods))
    fig, ax = plt.subplots(figsize=(14, 10))

    p = ax.bar(index - bar_width, avg_costs, bar_width, label='Average Cost')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    # p = ax.bar(index, avg_collisions, bar_width, label='Average Collisions')
    # ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index, avg_nodes, bar_width, label='Average Nodes')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index + bar_width, avg_planning_time, bar_width, label='Average Planning Time')
    ax.bar_label(p, fmt='{:,.0f}' + 's', fontsize=20)
    # p = ax.bar(index + bar_width * 2, avg_samples, bar_width, label='Average Samples')
    # ax.bar_label(p, fmt='{:,.0f}', fontsize=18)

    ax.set_xlabel('Method', fontsize="20")
    ax.set_ylabel('Average Value', fontsize="20")
    # ax.set_title('Average Metrics for Successful Tests by Method of ' + str(num_tests) + " Experiments", fontsize=20,
    #              fontweight="bold")
    ax.set_xticks(index)
    ax.set_xticklabels(methods, fontsize=20)
    ax.legend(fontsize=20)
    name = 'average_test_rrt'
    filename = get_name_to_save_plot(name_folder, name, path_solutions, '.svg')
    plt.savefig(filename, format='svg', dpi=500)
    plt.show()


def calculate_average_by_method_from_db(test_number=None):
    try:
        cnx = mysql.connector.connect(
            user='root',
            password='12345678',
            host='localhost',
            database='tesis'
        )
        cursor = cnx.cursor(dictionary=True)

        # Construir la consulta SQL
        query = """
            SELECT
                method,
                AVG(average_distance) AS avg_distance,
                AVG(std_dev_distance) AS avg_std_dev,
                AVG(max_distance) AS avg_max_distance,
                AVG(min_distance) AS avg_min_distance
            FROM
                results_tests_rrt
        """

        # Añadir cláusula WHERE si se proporciona test_number
        if test_number is not None:
            query += f" WHERE test_number = '{test_number}'"

        query += " GROUP BY method;"

        # Ejecutar la consulta SQL
        cursor.execute(query)
        result = cursor.fetchall()
        methods = [row['method'] for row in result]
        data_keys = ['avg_distance', 'avg_std_dev', 'avg_max_distance', 'avg_min_distance']
        data_values = [[row[key] for key in data_keys] for row in result]

        # Imprimir resultados en una tabla
        table = PrettyTable()
        table.field_names = ['Method', 'Average Distance', 'Std Dev Distance', 'Max Distance', 'Min Distance']

        for row in result:
            table.add_row([row['method']] + [round(row[key], 2) for key in
                                             ['avg_distance', 'avg_std_dev', 'avg_max_distance', 'avg_min_distance']])

        print(table)
        create_bar_chart(result)
        create_boxplot(result)
        create_violin_plot(result)
        create_line_plot(result)
        create_stacked_bar_chart(result)
        create_grouped_bar_chart(result)
        create_heatmap(result)
        create_scatter_plot(result)
        # create_radar_chart(result)

    except Exception as e:
        print(e)
    finally:
        # Cerrar la conexión a la base de datos
        if cnx.is_connected():
            cursor.close()
            cnx.close()


def create_scatter_plot(result):
    df = pd.DataFrame(result)
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x='avg_distance', y='avg_std_dev', hue='method', size='avg_max_distance', sizes=(20, 200), data=df)
    plt.title('Scatter Plot with Custom Colors and Sizes')
    plt.xlabel('Average Distance')
    plt.ylabel('Std Dev Distance')
    plt.show()


def create_heatmap(result):
    sns.set(style="whitegrid")
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    plt.figure(figsize=(12, 8))
    sns.heatmap(df, annot=True, cmap='viridis', linewidths=.5)
    plt.title('Heatmap of Average Metrics')
    plt.xlabel('Metric')
    plt.ylabel('Method')
    plt.show()


def create_stacked_bar_chart(result):
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    df.plot(kind='bar', stacked=True)
    plt.title('Stacked Bar Chart of Average Metrics')
    plt.xlabel('Method')
    plt.ylabel('Average Value')
    plt.show()


def create_grouped_bar_chart(result):
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    df.plot(kind='bar')
    plt.title('Grouped Bar Chart of Average Metrics')
    plt.xlabel('Method')
    plt.ylabel('Average Value')
    plt.show()


def create_bar_chart(result):
    df = pd.DataFrame(result)
    df.plot(x='method', y=['avg_distance'], kind='bar', rot=0, legend=False)
    plt.title('Average Distance Comparison')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


def create_boxplot(result):
    df = pd.DataFrame(result)
    sns.boxplot(x='method', y='avg_distance', data=df)
    plt.title('Boxplot of Average Distance')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


def create_violin_plot(result):
    df = pd.DataFrame(result)
    sns.violinplot(x='method', y='avg_distance', data=df)
    plt.title('Violin Plot of Average Distance')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


def create_line_plot(result):
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    df.plot(legend=True)
    plt.title('Line Plot of Average Distance Over Tests')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


def create_radar_chart(methods, data_values):
    # Normalizar los datos para que estén en el rango [0, 1]
    data_normalized = np.array(data_values) / np.array(data_values).max(axis=0)

    # Crear el gráfico radar
    angles = np.linspace(0, 2 * np.pi, len(data_values[0]), endpoint=False)
    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    ax.set_theta_offset(np.pi / 2)
    ax.set_theta_direction(-1)

    for i, label in enumerate(methods):
        values = data_normalized[i]
        values = np.concatenate((values, [values[0]]))  # Cerrar el círculo
        ax.plot(angles, values, label=label)
        ax.fill(angles, values, alpha=0.25)

    ax.set_yticklabels([])
    ax.set_xticks(angles[:-1])
    # ax.set_xticklabels(data_keys)
    plt.title('Radar Chart - Average Metrics by Method')
    plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

    # Mostrar el gráfico
    plt.show()


def average_curvature(test_number='20240219-07bd2315-a44a-4e0f-852d-3ca7275941b4'):
    try:
        cnx = mysql.connector.connect(
            user='root',
            password='12345678',
            host='localhost',
            database='tesis'
        )
        cursor = cnx.cursor(dictionary=True)

        # Construir la consulta SQL
        query = """
             SELECT
                 method,
                 AVG(smoothness) AS avg_smoothness,
                 AVG(curvature) AS avg_curvature
             FROM
                 results_tests_rrt
         """

        # Añadir cláusula WHERE si se proporciona test_number
        if test_number is not None:
            query += f" WHERE test_number = '{test_number}'"

        query += " GROUP BY method;"

        # Ejecutar la consulta SQL
        cursor.execute(query)
        result = cursor.fetchall()
        # Imprimir resultados en una tabla
        table = PrettyTable()
        table.field_names = ['Method', 'Avg Smoothness', 'Avg Curvature', 'Best Method (Suavidad)',
                             'Diff. Suavidad (%)']

        # Encontrar el método con la mejor suavidad y curvatura
        best_smoothness = min(result, key=lambda x: x['avg_smoothness'])

        for row in result:
            smoothness_improvement = ((best_smoothness['avg_smoothness'] - row['avg_smoothness']) / best_smoothness[
                'avg_smoothness']) * 100
            best_method_smoothness = 'Yes' if row['method'] == best_smoothness['method'] else 'No'

            table.add_row([
                row['method'],
                round(row['avg_smoothness'], 2),
                round(row['avg_curvature'], 5),
                best_method_smoothness,
                f"{smoothness_improvement:.2f}%"
            ])

        print(table)
    except Exception as e:
        print(e)
    finally:
        # Cerrar la conexión a la base de datos
        if cnx.is_connected():
            cursor.close()
            cnx.close()


if __name__ == "__main__":
    name_folder = create_folder.create_folder("../solutions")
    test_number = '20240130-c05af441-e181-4b95-885d-bfffd222bfa0'
    # test_number = '20240219-9d82c29e-99a1-493f-bd2f-e5f6b81e0332'

    path_solutions = '../solutions'
    # success_failure_rate_by_method(test_number, name_folder, path_solutions, 10)
    # average_test(test_number, name_folder, path_solutions, 10)
    # average_curvature(test_number)
    calculate_average_by_method_from_db(test_number)
