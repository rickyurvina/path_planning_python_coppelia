def path_cost_final(self, start_node, end_node):
    """
    Calculate the final cost of the path from the start node to the end node.

    Args:
        self: Instance of the RRT class.
        start_node (Node): Starting node of the path.
        end_node (Node): Ending node of the path.

    Returns:
        float: Final cost of the path from start to end.
    """
    cost = 0
    curr_node = end_node
    while start_node.row != curr_node.row or start_node.col != curr_node.col:
        # Keep tracing back until finding the start_node
        # or no path exists
        parent = curr_node.parent
        if parent is None:
            print("Invalid Path")
            return 0
        cost += curr_node.cost
        curr_node = parent

    return cost
