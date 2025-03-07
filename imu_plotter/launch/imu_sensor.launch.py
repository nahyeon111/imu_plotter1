import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='imu_plotter',
            executable='imu_publ',
            name='imu_publ',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='imu_plotter',
            executable='posvel',
            name='posvel',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='imu_plotter',
            executable='acceleration_pi_ro',
            name='acceleration_pi_ro',
            output='screen'
        ),


        launch_ros.actions.Node(
            package='imu_plotter',
            executable='angular_acceleration_ya_pi_ro',
            name='angular_acceleration_ya_pi_ro',
            output='screen'
        ),


        launch_ros.actions.Node(
            package='imu_plotter',
            executable='magnetic_ya',
            name='magnetic_ya',
            output='screen'
        ),


        launch_ros.actions.Node(
            package='imu_plotter',
            executable='kalman_filtter',
            name='kalman_filtter',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='imu_plotter',
            executable='visualizer',
            name='visualizer',
            output='screen'
        ),


    ])
