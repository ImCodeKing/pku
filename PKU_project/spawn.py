import rospy
import os

def add_model():
    GAZEBO_MODEL_PATH = '/home/j/model_editor_models/'
    # model_to_add = 'world_with_wall' # string
    model_to_add = 'april_car'
    os.system("rosrun gazebo_ros spawn_model -file " + GAZEBO_MODEL_PATH + model_to_add +"/model.sdf -sdf -model " + model_to_add + " -x 0 -y 0 -z 0.4")

def main():
    rospy.init_node('model_change', anonymous=True)
    add_model()


if __name__ == '__main__':
    main()