from datetime import datetime

import mysql.connector
import traceback
from colorama import init, Fore

from src.components import create_folder
from src.components.common.save_files import get_name_to_save_plot
from src.steps import config
import matplotlib.pyplot as plt
import numpy as np

init()


def average_test(test_number='20231112-d66cad06-6c3a-47e0-8234-99c2c428936a', name_folder='solution_825_12112023_1',
                 path_solutions='../../solutions', num_tests=1):
    conn = mysql.connector.connect(
        user='root',
        password='12345',
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
        password='12345',
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
    filename = get_name_to_save_plot(name_folder, name, path_solutions)
    plt.savefig(filename, dpi=500)
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
    p = ax.bar(index, avg_collisions, bar_width, label='Average Collisions')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index + bar_width, avg_nodes, bar_width, label='Average Nodes')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index + bar_width * 2, avg_planning_time, bar_width, label='Average Planning Time')
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
    filename = get_name_to_save_plot(name_folder, name, path_solutions)
    plt.savefig(filename, dpi=500)
    plt.show()


if __name__ == "__main__":
    name_folder = create_folder.create_folder("../solutions")
    test_number = '20231112-e95a987b-59fc-436e-b997-99d50167cfa5'
    path_solutions = '../solutions'
    success_failure_rate_by_method(test_number, name_folder, path_solutions, 10)
    average_test(test_number, name_folder, path_solutions, 10)
