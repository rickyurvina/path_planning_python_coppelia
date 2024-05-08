# Import necessary libraries
import mysql.connector
from colorama import init, Fore
from src.components import create_folder
from src.components.common.save_files import get_name_to_save_plot
import matplotlib.pyplot as plt
import numpy as np
from prettytable import PrettyTable
import seaborn as sns
import pandas as pd

from src.steps import config

# Initialize colorama for colored terminal text
init()


# Function to calculate average test results
def average_test(test_number='20240219-9d82c29e-99a1-493f-bd2f-e5f6b81e0332', name_folder='solution_825_12112023_1',
                 path_solutions='../../solutions', num_tests=1):
    # Connect to the database
    conn = mysql.connector.connect(
        user=config.MYSQL_USER,
        password=config.MYSQL_PASSWORD,
        host=config.MYSQL_HOST,
        database=config.MYSQL_DATABASE
    )

    # Create a cursor object
    cursor = conn.cursor()

    # SQL query to fetch average test results
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
    # Execute the query
    cursor.execute(query, (test_number,))

    # Fetch the results
    results = cursor.fetchall()

    # Plot the results
    plot_test_rrt(results, name_folder, path_solutions, num_tests)

    # Print the results
    for row in results:
        print(row)

    table = PrettyTable()

    # Agrega las columnas a la tabla
    table.field_names = ["Method", "Total Tests", "Average Total Cost", "Average Total Collisions",
                         "Average Total Planning Time", "Average Waypoints Number", "Average Min Iter",
                         "Average Max Iter", "Average Total Nodes", "Average Total Samples"]

    # Agrega los resultados a la tabla
    for row in results:
        table.add_row(row)

    # Imprime la tabla
    print(table)

    # Close the cursor and connection
    cursor.close()
    conn.close()


# Function to calculate success and failure rate by method
def success_failure_rate_by_method(test_number='20231112-d66cad06-6c3a-47e0-8234-99c2c428936a',
                                   name_folder='solution_825_12112023_1',
                                   path_solutions='../../solutions', num_tests=1):
    # Connect to the database
    conn = mysql.connector.connect(
        user=config.MYSQL_USER,
        password=config.MYSQL_PASSWORD,
        host=config.MYSQL_HOST,
        database=config.MYSQL_DATABASE
    )

    # Create a cursor object
    cursor = conn.cursor()

    # SQL query to fetch success and failure rate by method
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
    # Execute the query
    cursor.execute(query, (test_number,))

    # Fetch the results
    results = cursor.fetchall()

    # Initialize a dictionary to store success and failure rates
    method_success_failure = {}

    # Iterate over the results and calculate success and failure rates
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

    # Close the cursor and connection
    cursor.close()
    conn.close()

    # Print the success and failure rates
    for method, rates in method_success_failure.items():
        total_tests_method = rates['success'] + rates['failure']
        success_rate = (rates['success'] / total_tests_method) * 100
        failure_rate = (rates['failure'] / total_tests_method) * 100

        print(f"Method: {method}")
        print(f"  Success Rate: {success_rate}%")
        print(f"  Failure Rate: {failure_rate}%")
        print("---------------------")

    # Plot the success and failure rates
    plot_success_failure_bar_by_method(method_success_failure, name_folder, path_solutions, num_tests)


# Function to plot success and failure rates by method
def plot_success_failure_bar_by_method(method_success_failure, name_folder, path_solutions='../../solutions',
                                       num_tests=1):
    # Get the methods and success rates
    methods = list(method_success_failure.keys())
    success_rates = [(rates['success'] / (rates['success'] + rates['failure'])) * 100 for rates in
                     method_success_failure.values()]
    failure_rates = 100 - np.array(success_rates)

    # Create a bar plot
    fig, ax = plt.subplots(1, figsize=(12, 10))

    bar_width = 0.35
    index = np.arange(len(methods))

    bars1 = ax.bar(index, success_rates, bar_width, label='Success', color='green')
    ax.bar_label(bars1, fmt='{:,.0f}' + '%', fontsize=20)

    bars2 = ax.bar(index + bar_width, failure_rates, bar_width, label='Failure', color='red')
    ax.bar_label(bars2, fmt='{:,.0f}' + '%', fontsize=20)

    ax.set_xlabel('Method', fontsize="18")
    ax.set_ylabel('Percentage', fontsize="18")
    ax.set_xticks(index + bar_width / 2)
    ax.set_xticklabels(methods, fontsize="18")
    yticks = np.arange(0, 101, 20)
    ax.set_yticks(yticks)
    ax.set_yticklabels([f'{val}%' for val in yticks], fontsize="18")

    # Adjust the position of the legend
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1), fontsize=20, fancybox=True, shadow=True, ncol=2)
    ax.set_ylim(0, 100)

    # Save the plot
    name = 'success_failure_rate_by_method'
    filename = get_name_to_save_plot(name_folder, name, path_solutions, '.svg')
    plt.savefig(filename, format='svg', dpi=500)
    plt.show()


# Function to plot average test results
def plot_test_rrt(results, name_folder, path_solutions='../../solutions', num_tests=10):
    # Get the methods and average results
    methods = [row[0] for row in results]
    num_tests = results[0][1]
    avg_costs = [row[2] for row in results]
    avg_collisions = [row[3] for row in results]
    avg_planning_time = [row[4] for row in results]
    avg_nodes = [row[8] for row in results]
    avg_samples = [row[9] for row in results]

    # Create a bar plot
    bar_width = 0.2
    index = np.arange(len(methods))
    fig, ax = plt.subplots(figsize=(14, 10))

    p = ax.bar(index - bar_width, avg_costs, bar_width, label='Average Cost')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index, avg_nodes, bar_width, label='Average Nodes')
    ax.bar_label(p, fmt='{:,.0f}', fontsize=20)
    p = ax.bar(index + bar_width, avg_planning_time, bar_width, label='Average Planning Time')
    ax.bar_label(p, fmt='{:,.0f}' + 's', fontsize=20)

    ax.set_xlabel('Method', fontsize="20")
    ax.set_ylabel('Average Value', fontsize="20")
    ax.set_xticks(index)
    ax.set_xticklabels(methods, fontsize=20)
    ax.legend(fontsize=20)

    # Save the plot
    name = 'average_test_rrt'
    # filename = get_name_to_save_plot(name_folder, name, path_solutions, '.svg')
    # plt.savefig(filename, format='svg', dpi=500)
    plt.show()


# Function to calculate average metrics by method from the database
def calculate_average_by_method_from_db(test_number=None):
    try:
        # Connect to the database
        cnx = mysql.connector.connect(
            user=config.MYSQL_USER,
            password=config.MYSQL_PASSWORD,
            host=config.MYSQL_HOST,
            database=config.MYSQL_DATABASE
        )
        cursor = cnx.cursor(dictionary=True)

        # Build the SQL query
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

        # Add WHERE clause if test_number is provided
        if test_number is not None:
            query += f" WHERE test_number = '{test_number}'"

        query += " GROUP BY method;"

        # Execute the SQL query
        cursor.execute(query)
        result = cursor.fetchall()
        data_keys = ['avg_distance', 'avg_std_dev', 'avg_max_distance', 'avg_min_distance']

        # Print results in a table
        table = PrettyTable()
        table.field_names = ['Method', 'Average Distance', 'Std Dev Distance', 'Max Distance', 'Min Distance']

        for row in result:
            table.add_row([row['method']] + [round(row[key], 2) for key in
                                             ['avg_distance', 'avg_std_dev', 'avg_max_distance', 'avg_min_distance']])

        print(table)
    except Exception as e:
        print(e)
    finally:
        # Close the database connection
        if cnx.is_connected():
            cursor.close()
            cnx.close()


# Function to create a scatter plot
def create_scatter_plot(result):
    df = pd.DataFrame(result)
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x='avg_distance', y='avg_std_dev', hue='method', size='avg_max_distance', sizes=(20, 200), data=df)
    plt.title('Scatter Plot with Custom Colors and Sizes')
    plt.xlabel('Average Distance')
    plt.ylabel('Std Dev Distance')
    plt.show()


# Function to create a heatmap
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


# Function to create a stacked bar chart
def create_stacked_bar_chart(result):
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    df.plot(kind='bar', stacked=True)
    plt.title('Stacked Bar Chart of Average Metrics')
    plt.xlabel('Method')
    plt.ylabel('Average Value')
    plt.show()


# Function to create a grouped bar chart
def create_grouped_bar_chart(result):
    df = pd.DataFrame(result)
    df.set_index('method', inplace=True)
    df.plot(kind='bar')
    plt.title('Grouped Bar Chart of Average Metrics')
    plt.xlabel('Method')
    plt.ylabel('Average Value')
    plt.show()


# Function to create a bar chart
def create_bar_chart(result):
    df = pd.DataFrame(result)
    df.plot(x='method', y=['avg_distance'], kind='bar', rot=0, legend=False)
    plt.title('Average Distance Comparison')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


# Function to create a boxplot
def create_boxplot(result):
    df = pd.DataFrame(result)
    sns.boxplot(x='method', y='avg_distance', data=df)
    plt.title('Boxplot of Average Distance')
    plt.ylabel('Average Distance')
    plt.xlabel('Method')
    plt.show()


# Function to create a violin plot
def create_violin_plot(result):
    df = pd.DataFrame(result)


if __name__ == '__main__':
    average_test()
