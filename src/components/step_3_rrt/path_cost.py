def path_cost(self, start_node, end_node, old_cost=0):
    """
    Calculate the cost of the path from the start node to the end node.

    Args:
        self: Instance of the RRT class.
        start_node (Node): Starting node of the path.
        end_node (Node): Ending node of the path.
        old_cost (float, optional): Old cost to compare with. Defaults to 0.

    Returns:
        float: Cost of the path from start to end.
    """
    cost = 0
    curr_node = end_node
    while round(start_node.row, 3) != round(curr_node.row, 3) or round(start_node.col, 3) != round(curr_node.col, 3):
        parent = curr_node.parent
        if parent is None:
            print("Invalid Path")
            return 0
        cost += curr_node.cost
        curr_node = parent
        if old_cost > cost:
            return old_cost

    return cost
