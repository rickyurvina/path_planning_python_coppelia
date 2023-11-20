def path_cost(self, start_node, end_node, old_cost=0):
    cost = 0
    curr_node = end_node
    npi = 0
    while round(start_node.row, 3) != round(curr_node.row, 3) or round(start_node.col, 3) != round(curr_node.col,
                                                                                                   3):
        # print("Path cost iteration: ", npi)
        npi = npi + 1
        parent = curr_node.parent
        if parent is None:
            print("Invalid Path")
            return 0
        cost += curr_node.cost
        curr_node = parent
        if old_cost > cost:
            return old_cost

    return cost
