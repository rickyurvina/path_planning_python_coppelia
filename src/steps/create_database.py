import config
import mysql.connector
from mysql.connector import Error


def create_database():
    """
    This function creates a new database in MySQL. If the database already exists, it drops it and creates a new one.
    """
    try:
        # Establish a connection to MySQL
        connection = mysql.connector.connect(
            host=config.MYSQL_HOST,  # MySQL server address
            user=config.MYSQL_USER,  # MySQL username
            password=config.MYSQL_PASSWORD  # MySQL password
        )

        if connection.is_connected():
            cursor = connection.cursor()

            # Check if the database exists
            cursor.execute(f"SHOW DATABASES LIKE '{config.MYSQL_DATABASE}'")
            result = cursor.fetchone()

            # If the database exists, drop it
            if result:
                cursor.execute(f"DROP DATABASE {config.MYSQL_DATABASE}")
                print("Existing database successfully dropped")

            # Create the database
            cursor.execute(f"CREATE DATABASE {config.MYSQL_DATABASE}")
            print("Database successfully created")
    except Error as e:
        print("Error connecting to MySQL", e)
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()
            print("MySQL connection closed")


def execute_sql_file(filename):
    """
    This function reads an SQL file and executes each SQL command in the file.
    """
    # Read the SQL file
    with open(filename, 'r') as file:
        sql_file = file.read()

    # Split the SQL file into individual commands
    sql_commands = sql_file.split(';')

    try:
        # Establish a connection to MySQL
        connection = mysql.connector.connect(
            host=config.MYSQL_HOST,  # MySQL server address
            user=config.MYSQL_USER,  # MySQL username
            password=config.MYSQL_PASSWORD,  # MySQL password
            database=config.MYSQL_DATABASE  # MySQL database name
        )

        if connection.is_connected():
            cursor = connection.cursor()

            # Execute each SQL command
            for command in sql_commands:
                try:
                    if command.strip() != '':
                        cursor.execute(command)
                except Error as e:
                    print(f"Error executing SQL command '{command}':", e)

            print("SQL file successfully executed")
    except Error as e:
        print("Error connecting to MySQL", e)
    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()
            print("MySQL connection closed")


if __name__ == "__main__":
    # If this script is run as the main program, it creates a new database and executes an SQL file
    create_database()
    execute_sql_file('../database/dump_tesis.sql')
