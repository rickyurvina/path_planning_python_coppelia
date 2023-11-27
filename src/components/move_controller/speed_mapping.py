def speed_mapping(vrel, wrel):
    fv = 0.4273
    fw = 0.1504

    if wrel >= 0:  # if turning left
        vright = fv * vrel - wrel * fw
        vleft = fv * vrel + wrel * fw
    else:  # if turning right
        vright = fv * vrel - wrel * fw
        vleft = fv * vrel + wrel * fw

    return vright, vleft


if __name__ == '__main__':
    # Example usage:
    vrel_input = 0.2
    wrel_input = 0.2
    vright_output, vleft_output = speed_mapping(vrel_input, wrel_input)
    print("vright:", vright_output)
    print("vleft:", vleft_output)
