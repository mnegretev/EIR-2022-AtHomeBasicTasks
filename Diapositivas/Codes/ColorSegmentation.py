def segment_by_color(img_bgr, points, obj_name):
    # Para 'pringles': [25, 50, 50] - [35, 255, 255]
    # Para 'drink':    [10,200, 50] - [20, 255, 255]
    lower_limit = numpy.array([25, 50, 50])
    upper_limit = numpy.array([35,255,255])
