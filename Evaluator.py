# coding: utf-8

from ctypes import *

"""

The origin of the coordinate is at the corner of the ROI

"""

class Evaluator(object):
    """Creates a VSR solver object"""

    def __init__(self, roi_shape=[10, 10, 10], dead_zone=[0,0,0,0,0,0], cube_resolution=[1, 1, 1], lidar_num=1,
                 laser_num=2,pitch_angle=[-10, 10]):
        """

        Parameters:
        ----------
            @param list roi_shape : ROI geometric shape [x,y,z] (m)
            @param list dead_zone : dead zone geometric shape [x,y,z] (m), the dead zone will be excluded from ROI
            @param list cube_resolution : segmented cubes'resolution [x,y,z] (m)
            @param int lidar_num : number of lidars
            @param int laser_num : number of lasers
            @param list pitch_angle : each laser's pitch angle, from low to high (°)

        """
        self.evaluator = cdll.LoadLibrary("./Evaluator/build/libevaluator.so")
        self.evaluator.subspace_segmentation.restype = c_float

        assert (len(roi_shape) == 3), "input parameter roi_shape is incorrect"
        assert (len(dead_zone) == 6), "input parameter dead_zone is incorrect"
        assert (len(cube_resolution) == 3), "input parameter cube_resolution is incorrect"
        assert (len(pitch_angle) == laser_num), "input parameter laser_num's length must equals laser_num"

        self.cube_x_num = roi_shape[0] / cube_resolution[0]  # The number of cubers along x axis
        self.cube_y_num = roi_shape[1] / cube_resolution[1]  # The number of cubers along y axis
        self.cube_z_num = roi_shape[2] / cube_resolution[2]  # The number of cubers along z axis

        assert (self.cube_x_num == int(self.cube_x_num)), "The x length of ROI and x cube resolution do not match"
        assert (self.cube_y_num == int(self.cube_y_num)), "The y length of ROI and y cube resolution do not match"
        assert (self.cube_z_num == int(self.cube_z_num)), "The z length of ROI and z cube resolution do not match"

        self.cube_x_num = int(self.cube_x_num)
        self.cube_y_num = int(self.cube_y_num)
        self.cube_z_num = int(self.cube_z_num)

        self.cube_resolution = cube_resolution
        self.dead_zone=dead_zone

        self.lidar_num = lidar_num
        self.laser_num = laser_num

        pitch_angle.sort()  # from low to high
        self.pitch_angle = pitch_angle

        self.subspace_num_max = (laser_num + 1) ** lidar_num  # The maximum number of the subspaces

    def solve(self, lidar_origin=[0, 0, 0,0,0,0]):
        """

        Parameters:
        ----------
            @param list lidar_origin : lidar sensors origin [x1,y1,z1,x2,y2,z2 ...] (m)

        Return:
        ----------
            @param floar re[0] : maximum VSR
            @param floar re[1] : minimum VSR
            @param floar re[2] : stdev VSR
            @param floar re[3] : average VSR
        """

        assert (len(
            lidar_origin) == 6 * self.lidar_num), "The length of lidar_origin must equals 6 times of the number of lidars [x1,y1,z1,roll,pitch,yaw...]"

        int_size = c_int * 3
        cube_num_send = int_size()
        cube_num_send[0] = c_int(self.cube_x_num)
        cube_num_send[1] = c_int(self.cube_y_num)
        cube_num_send[2] = c_int(self.cube_z_num)

        float_size = c_float * 3
        cube_resolution_send = float_size()
        for i in range(3):
            cube_resolution_send[i] = c_float(self.cube_resolution[i])

        float_size = c_float * len(lidar_origin)
        lidar_origin_send = float_size()
        for i in range(len(lidar_origin)):
            lidar_origin_send[i] = c_float(lidar_origin[i])

        float_size = c_float * len(self.pitch_angle)
        pitch_angle_send = float_size()
        for i in range(len(self.pitch_angle)):
            pitch_angle_send[i] = c_float(self.pitch_angle[i])


        float_size = c_float * 4
        result = float_size()
        for i in range(len(result)):
            result[i] = c_float(-1)

        int_size = c_int* 6
        dead_zone_range = int_size()
        dead_zone_range[0] = c_int(int(self.dead_zone[0] / self.cube_resolution[0]))  #x_dead_low
        dead_zone_range[1] = c_int(int(self.dead_zone[1] / self.cube_resolution[0]))  #x_dead_high
        dead_zone_range[2] = c_int(int(self.dead_zone[2] / self.cube_resolution[1]))  #y_dead_low
        dead_zone_range[3] = c_int(int(self.dead_zone[3] / self.cube_resolution[1]))  #y_dead_high
        dead_zone_range[4] = c_int(int(self.dead_zone[4] / self.cube_resolution[2]))  #z_dead_low
        dead_zone_range[5] = c_int(int(self.dead_zone[5] / self.cube_resolution[2]))  #z_dead_high

        self.evaluator.subspace_segmentation(cube_num_send, cube_resolution_send,dead_zone_range,
                                                    lidar_origin_send, c_int(self.lidar_num), c_int(self.laser_num),
                                                    c_long(self.subspace_num_max), pitch_angle_send,result)
        re = []  # max, min,stdev,mean
        for i in range(len(result)):
            re.append(result[i])
        return re[0]

class ROISubspaceOccupyNumber(object):
    """Creates a inscribed sphere solver object"""

    def __init__(self, roi_shape=[10, 10, 10],lidar_origin=[0, 0, 0], dead_zone=[0,0,0,0,0,0], cube_resolution=[1, 1, 1], lidar_num=1,
                 laser_num=2,pitch_angle=[-10, 10],obj_size=[1,1,1]):
        """

        Parameters:
        ----------
            @param list roi_shape : ROI geometric shape [x,y,z] (m)
            @param list dead_zone : dead zone geometric shape [x,y,z] (m), the dead zone will be excluded from ROI
            @param list cube_resolution : segmented cubes'resolution [x,y,z] (m)
            @param int lidar_num : number of lidars
            @param int laser_num : number of lasers
            @param list pitch_angle : each laser's pitch angle, from low to high (°)
            @param list obj_size : the object size (m)

        """
        self.evaluator = cdll.LoadLibrary("./Evaluator/build/libevaluator.so")
        self.evaluator.ROI_occupy_subspace_num.restype = c_int
        assert (len(lidar_origin) == 6 * lidar_num), "The length of lidar_origin must equals 6 times of the number of lidars [x1,y1,z1,roll,pitch,yaw...]"
        assert (len(roi_shape) == 3), "input parameter roi_shape is incorrect"
        assert (len(obj_size) == 3), "input parameter roi_shape is incorrect"


        assert (len(dead_zone) == 6), "input parameter dead_zone is incorrect"
        assert (len(cube_resolution) == 3), "input parameter cube_resolution is incorrect"
        assert (len(pitch_angle) == laser_num), "input parameter laser_num's length must equals laser_num"

        self.cube_x_num = obj_size[0] / cube_resolution[0]  # The number of cubers along x axis
        self.cube_y_num = obj_size[1] / cube_resolution[1]  # The number of cubers along y axis
        self.cube_z_num = obj_size[2] / cube_resolution[2]  # The number of cubers along z axis


        self.cube_x_max = roi_shape[0] / cube_resolution[0] - self.cube_x_num
        self.cube_y_max = roi_shape[1] / cube_resolution[1] - self.cube_y_num
        self.cube_z_max = roi_shape[2] / cube_resolution[2] - self.cube_z_num

        self.lidar_origin=lidar_origin

        assert (self.cube_x_num == int(self.cube_x_num)), "The x length of ROI and x cube resolution do not match"
        assert (self.cube_y_num == int(self.cube_y_num)), "The y length of ROI and y cube resolution do not match"
        assert (self.cube_z_num == int(self.cube_z_num)), "The z length of ROI and z cube resolution do not match"

        self.cube_x_num = int(self.cube_x_num)
        self.cube_y_num = int(self.cube_y_num)
        self.cube_z_num = int(self.cube_z_num)

        self.cube_resolution = cube_resolution
        self.dead_zone=dead_zone

        self.lidar_num = lidar_num
        self.laser_num = laser_num

        pitch_angle.sort()  # from low to high
        self.pitch_angle = pitch_angle

        self.subspace_num_max = (laser_num + 1) ** lidar_num  # The maximum number of the subspaces


    def solve(self,start_point):
        """

        Parameters:
        ----------
            @param list start_point : the object's position (m)

        Return:
        ----------
            @param int subspace_count : how many subspace the object occupied with this start point

        """

        int_size = c_int * 3
        cube_num_send = int_size()
        cube_num_send[0] = c_int(self.cube_x_num)
        cube_num_send[1] = c_int(self.cube_y_num)
        cube_num_send[2] = c_int(self.cube_z_num)

        int_size = c_int * 3
        start_point_send = int_size()
        start_point_send[0] = c_int(start_point[0])
        start_point_send[1] = c_int(start_point[1])
        start_point_send[2] = c_int(start_point[2])

        float_size = c_float * 3
        cube_resolution_send = float_size()
        for i in range(3):
            cube_resolution_send[i] = c_float(self.cube_resolution[i])

        float_size = c_float * len(self.lidar_origin)
        lidar_origin_send = float_size()
        for i in range(len(self.lidar_origin)):
            lidar_origin_send[i] = c_float(self.lidar_origin[i])

        float_size = c_float * len(self.pitch_angle)
        pitch_angle_send = float_size()
        for i in range(len(self.pitch_angle)):
            pitch_angle_send[i] = c_float(self.pitch_angle[i])


        int_size = c_int* 6
        dead_zone_range = int_size()
        dead_zone_range[0] = int(self.dead_zone[0] / self.cube_resolution[0])  #x_dead_low
        dead_zone_range[1] = int(self.dead_zone[1] / self.cube_resolution[0])  #x_dead_high
        dead_zone_range[2] = int(self.dead_zone[2] / self.cube_resolution[1])  #y_dead_low
        dead_zone_range[3] = int(self.dead_zone[3] / self.cube_resolution[1])  #y_dead_high
        dead_zone_range[4] = int(self.dead_zone[4] / self.cube_resolution[2])  #z_dead_low
        dead_zone_range[5] = int(self.dead_zone[5] / self.cube_resolution[2])  #z_dead_high

        subspace_count = self.evaluator.ROI_occupy_subspace_num(cube_num_send, cube_resolution_send,dead_zone_range,
                                                    lidar_origin_send, c_int(self.lidar_num), c_int(self.laser_num),
                                                    c_long(self.subspace_num_max), pitch_angle_send,start_point_send)

        return subspace_count


def logConfig(config):
    print("*********************************************")
    print("Your configuration: ")
    print("roi_shape: " ,config['roi_shape'])
    print("dead_zone: ", config['dead_zone'])
    print("cube_resolution: ", config['cube_resolution'])
    print("laser_num: ", config['laser_num'])
    print("lidar_num: ", config['lidar_num'])
    print("beam_angle: ", config['beam_angle'])
    print("max_itrs: ",config['max_itrs'])
    print("lower_bound: ", config['lower_bound'])
    print("upper_bound: ", config['upper_bound'])
    print("numb_bees: ", config['numb_bees'])
    print("estimate_solve_time: ", config['estimate_solve_time'])
    print("save_result_to_json: ", config['save_result_to_json'])
    print("*********************************************")

def estimateTime(config,VSR_solver,Hive):
    lidar_num = config['lidar_num']
    max_itrs = config['max_itrs']
    numb_bees = config['numb_bees']
    lower_bound = config['lower_bound'] * lidar_num
    upper_bound = config['upper_bound'] * lidar_num

    max_itrs_time_test = 3
    if numb_bees>100:
        numb_bees_time_test = 50
    else:
        numb_bees_time_test=numb_bees

    import time
    now = time.time()
    model = Hive.BeeHive(lower=lower_bound,
                         upper=upper_bound,
                         fun=VSR_solver.solve,
                         numb_bees=numb_bees_time_test,
                         max_itrs=max_itrs_time_test,
                         verbose=True)
    # runs model
    model.run()
    time_consume = time.time() - now
    time_estimate = time_consume * max_itrs * numb_bees / max_itrs_time_test / numb_bees_time_test
    print("The estimated time to solve: ", str(round(time_estimate,4)) + ' seconds')
    print("The estimated time to solve: ", str(round(time_estimate / 3600,4)) + ' hours')



def saveResults(config, model, total_time,cost):
    try:
        import json
    except:
        raise ImportError("json module not installed.")
    roi_shape = config['roi_shape']
    dead_zone = config['dead_zone']
    cube_resolution = config['cube_resolution']
    laser_num = config['laser_num']
    lidar_num = config['lidar_num']
    beam_angle = config['beam_angle']
    max_itrs = config['max_itrs']
    numb_bees = config['numb_bees']
    lower_bound = config['lower_bound'] * lidar_num
    upper_bound = config['upper_bound'] * lidar_num

    name = "lidar_" + str(lidar_num) + "_laser_" + str(laser_num) + '_' + str(1)
    solution = model.solution
    dict_data = {"name": name, "lidar_num": lidar_num, "laser_num": laser_num, "cube_resolution": cube_resolution,
                 "roi_shape": roi_shape, "dead_zone": dead_zone, "max_itrs": max_itrs, "numb_bees": numb_bees,
                 "lower": lower_bound[:6], "upper": upper_bound[:6], "beam_angle": list(beam_angle), "solution": solution,
                 "best_cost": model.best, "time_consume": total_time}
    with open('Result/data.json', 'a+') as f:
        json.dump(dict_data, f,indent=True)
        f.write('\n\n')

    dict_data2 = {"name": name, "lidar_num": lidar_num, "laser_num": laser_num,
                  "cube_resolution": cube_resolution, "cost": cost}
    with open('Result/data_cost.json', 'a+') as f:
        json.dump(dict_data2, f)
        f.write('\n\n')



'''
def timeTest(config,VSR_solver,Hive):
    lidar_num = config['lidar_num']
    max_itrs = config['max_itrs']
    numb_bees = config['numb_bees']
    lower_bound = config['lower_bound'] * lidar_num
    upper_bound = config['upper_bound'] * lidar_num

    max_itrs_time_test = 3

    
    import time
    bee_num_list=[]
    time_list=[]
    for i in range(25):
        if i ==0:
            continue
        bee_num_list.append(i * 2)
        numb_bees_time_test = i * 2
        now = time.time()
        model = Hive.BeeHive(lower=lower_bound,
                             upper=upper_bound,
                             fun=VSR_solver.solve,
                             numb_bees=numb_bees_time_test,
                             max_itrs=max_itrs_time_test,
                             verbose=True)
        # runs model
        model.run()
        time_consume = (time.time() - now)/max_itrs_time_test/numb_bees_time_test
        time_list.append(time_consume)
        print('******************************************************')
        print(i)
        print('******************************************************')
    import json
    dict_data = {"bee_num_list": bee_num_list, "time_list": time_list}
    with open('Result/time.json', 'a+') as f:
        json.dump(dict_data, f)
        f.write('\n')
    '''