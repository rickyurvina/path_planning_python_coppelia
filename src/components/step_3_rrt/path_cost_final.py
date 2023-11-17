def path_cost_final(self, start_node, end_node):
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
