
ROOM_WIDTH = 2
ROOM_HEIGHT = 2

def scale_pose_room_to_sim(x, y, scale_factor=1, origin_sim_x="CENTER", origin_sim_y="CENTER"):
    x_new  = x
    y_new = y

    # -> Adjust x origin
    if origin_sim_x == "CENTER":
        pass

    elif origin_sim_x == "LEFT":
        x_new += ROOM_WIDTH
    
    elif origin_sim_x == "RIGHT":
        x_new -= ROOM_WIDTH

    else:
        print("!!!!!!!!!!!!! INVALID X ORIGIN !!!!!!!!!!!!!")

    # -> Adjust y origin
    if origin_sim_y == "CENTER":
        pass

    elif origin_sim_y == "BOTTOM":
        y_new += ROOM_HEIGHT

    elif origin_sim_y == "TOP":
        y_new -= ROOM_HEIGHT

    else:
        print("!!!!!!!!!!!!! INVALID Y ORIGIN !!!!!!!!!!!!!")

    # -> Apply scaling factor
    x_new *= scale_factor
    y_new *= scale_factor


    return x_new, y_new

def scale_pose_sim_to_room(x, y, scale_factor=1, origin_sim_x="CENTER", origin_sim_y="CENTER"):
    x_new  = x
    y_new = y

    # -> Apply scaling factor
    x_new *= scale_factor
    y_new *= scale_factor

    # -> Adjust x origin
    if origin_sim_x == "CENTER":
        pass

    elif origin_sim_x == "LEFT":
        x_new -= ROOM_WIDTH
    
    elif origin_sim_x == "RIGHT":
        x_new += ROOM_WIDTH

    else:
        print("!!!!!!!!!!!!! INVALID X ORIGIN !!!!!!!!!!!!!")

    # -> Adjust y origin
    if origin_sim_y == "CENTER":
        pass

    elif origin_sim_y == "BOTTOM":
        y_new -= ROOM_HEIGHT

    elif origin_sim_y == "TOP":
        y_new -= ROOM_HEIGHT

    else:
        print("!!!!!!!!!!!!! INVALID Y ORIGIN !!!!!!!!!!!!!")

    return x_new, y_new

if __name__ == "__main__":
    x = 0.67
    y = 0

    print(x, y)

    x, y = scale_pose_room_to_sim(
        x=x,
        y=y,
        scale_factor=201/2,
        origin_sim_x="LEFT",
        origin_sim_y="BOTTOM"
    )

    print(x, y)

    x, y = scale_pose_sim_to_room(
        x=x,
        y=y,
        scale_factor=2/201,
        origin_sim_x="LEFT",
        origin_sim_y="BOTTOM"
    )

    print(x, y)
