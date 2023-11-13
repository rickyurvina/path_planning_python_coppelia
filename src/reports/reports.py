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

    results = cursor.fetchall()
    plot_test_rrt(results, name_folder, path_solutions, num_tests)

    for row in results:
        print(row)

    cursor.close()
    conn.close()


def plot_test_rrt(results, name_folder, path_solutions='../../solutions', num_tests=10):
    methods = [row[0] for row in results]
    avg_costs = [row[2] for row in results]
    avg_collisions = [row[3] for row in results]
    avg_nodes = [row[8] for row in results]
    avg_samples = [row[9] for row in results]

    bar_width = 0.2
    index = np.arange(len(methods))
    fig, ax = plt.subplots(figsize=(10, 6))

    p = ax.bar(index - bar_width, avg_costs, bar_width, label='Average Cost')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=9)
    p = ax.bar(index, avg_collisions, bar_width, label='Average Collisions')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=9)
    p = ax.bar(index + bar_width, avg_nodes, bar_width, label='Average Nodes')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=9)
    p = ax.bar(index + bar_width * 2, avg_samples, bar_width, label='Average Samples')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=9)

    ax.set_xlabel('Method', fontsize="14")
    ax.set_ylabel('Average Value', fontsize="14")
    ax.set_title('Average Metrics for Successful Tests by Method of: ' + str(num_tests) + " Experiments", fontsize=14,
                 fontweight="bold")
    ax.set_xticks(index)
    ax.set_xticklabels(methods, fontsize=14)
    ax.legend()
    name = 'average_test_rrt'
    filename = get_name_to_save_plot(name_folder, name, path_solutions)
    plt.savefig(filename, dpi=500)
    plt.show()


if __name__ == "__main__":
    name_folder = create_folder.create_folder("../solutions")
    test_number = '20231112-b5f9cc98-3c71-4846-8e62-90f1967df136'
    path_solutions = '../solutions'
    average_test(test_number, name_folder, path_solutions, 10)
