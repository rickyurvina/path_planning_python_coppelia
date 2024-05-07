-- MariaDB dump 10.19  Distrib 10.4.28-MariaDB, for osx10.10 (x86_64)
--
-- Host: 127.0.0.1    Database: tesis
-- ------------------------------------------------------
-- Server version	8.3.0

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `results`
--

DROP TABLE IF EXISTS `results`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `results`
(
    `id`                 int NOT NULL AUTO_INCREMENT,
    `total_time`         float        DEFAULT NULL,
    `load_data_time`     float        DEFAULT NULL,
    `tsp_time`           float        DEFAULT NULL,
    `rrt_time`           float        DEFAULT NULL,
    `folder`             varchar(255) DEFAULT NULL,
    `vehicle_capacities` int          DEFAULT NULL,
    `num_rows`           int          DEFAULT NULL,
    `expand_distance`    float        DEFAULT NULL,
    `goal_sample_rate`   float        DEFAULT NULL,
    `min_iter`           int          DEFAULT NULL,
    `max_iter`           int          DEFAULT NULL,
    `radius`             float        DEFAULT NULL,
    `method`             varchar(255) DEFAULT NULL,
    `path_length`        float        DEFAULT NULL,
    `total_loaded_tsp`   int          DEFAULT NULL,
    `total_length_tsp`   float        DEFAULT NULL,
    `on_line`            tinyint(1) DEFAULT NULL,
    PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=8 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Table structure for table `results_rrt`
--

DROP TABLE IF EXISTS `results_rrt`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `results_rrt`
(
    `id`                  int NOT NULL AUTO_INCREMENT,
    `method`              varchar(255) DEFAULT NULL,
    `test_number`         int          DEFAULT NULL,
    `total_cost`          float        DEFAULT NULL,
    `total_collisions`    int          DEFAULT NULL,
    `total_planning_time` float        DEFAULT NULL,
    `total_samples`       int          DEFAULT NULL,
    `waypoints_number`    int          DEFAULT NULL,
    `min_iter`            int          DEFAULT NULL,
    `max_iter`            int          DEFAULT NULL,
    `date`                timestamp NULL DEFAULT NULL,
    `name_folder`         varchar(255) DEFAULT NULL,
    `success`             tinyint(1) DEFAULT NULL,
    `total_nodes`         int          DEFAULT NULL,
    `extend_dis`          float        DEFAULT NULL,
    `neighbor_size`       int          DEFAULT NULL,
    `time_limit`          int          DEFAULT NULL,
    `average_distance`    float        DEFAULT NULL,
    `std_dev_distance`    float        DEFAULT NULL,
    `max_distance`        float        DEFAULT NULL,
    `min_distance`        float        DEFAULT NULL,
    `variance_distance`   float        DEFAULT NULL,
    `smoothness`          float        DEFAULT NULL,
    `curvature`           float        DEFAULT NULL,
    PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Table structure for table `results_tests_rrt`
--

DROP TABLE IF EXISTS `results_tests_rrt`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `results_tests_rrt`
(
    `id`                  int NOT NULL AUTO_INCREMENT,
    `method`              varchar(255) DEFAULT NULL,
    `test_number`         varchar(255) DEFAULT NULL,
    `total_cost`          float        DEFAULT NULL,
    `total_collisions`    int          DEFAULT NULL,
    `total_planning_time` float        DEFAULT NULL,
    `total_samples`       int          DEFAULT NULL,
    `waypoints_number`    int          DEFAULT NULL,
    `min_iter`            int          DEFAULT NULL,
    `max_iter`            int          DEFAULT NULL,
    `date`                timestamp NULL DEFAULT NULL,
    `name_folder`         varchar(255) DEFAULT NULL,
    `success`             tinyint(1) DEFAULT NULL,
    `total_nodes`         int          DEFAULT NULL,
    `extend_dis`          float        DEFAULT NULL,
    `neighbor_size`       int          DEFAULT NULL,
    `time_limit`          int          DEFAULT NULL,
    `average_distance`    float        DEFAULT NULL,
    `std_dev_distance`    float        DEFAULT NULL,
    `max_distance`        float        DEFAULT NULL,
    `min_distance`        float        DEFAULT NULL,
    `variance_distance`   float        DEFAULT NULL,
    `smoothness`          varchar(55)  DEFAULT NULL,
    `curvature`           varchar(55)  DEFAULT NULL,
    PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=1390 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping routines for database config.MYSQL_DATABASE
--
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2024-05-07 13:18:34
