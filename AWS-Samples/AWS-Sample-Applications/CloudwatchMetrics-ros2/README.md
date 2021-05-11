### Introduction
The cloudwatch_metrics_collector ROS Node publishes your robot's metrics to the cloud to enable you to easily track the health of a fleet with the use of automated monitoring and actions for when a robot's metrics show abnormalities.

The CloudwatchMetrics-ros2 node requires an IAM User with the cloudwatch:PutMetricData permission policy.  

1. Add “cloudwatch:PutMetricData” permission to the IAM user

    i) Select the IAM user created earlier, click Add permission, then tap Attach existing policies directly and then select the Create policy and paste the JSON snippet given below.
    ```
        {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Action": [
                      "cloudwatch:PutMetricData"
                    ],
                    "Resource": [
                        "*"
                    ]
                }
            ]
        }
    ```

    ii) Give name to policy, go to _Review policy_ and select _Create policy._
      
    iii) Add the created policy to the IAM user by searching the policy name. Follow the steps below.
    Go to created user -> **Add permission -> Attach existing policies directly -> search and select the created policy -> Next:review -> Add permission**
    Once the permission is set, open a terminal in Qualcomm Robotics RB5 follow the steps below
    
2. Clone the cloudwatchmetrics-ros2 app
- Create a ROS workspace and a source directory
    ```
    mkdir -p ~/ros-workspace/src 
    cd ~/ros-workspace/src
    ```
- Clone the cloudwatchmetrics-ros2 app into the source directory, install dependencies
    ```
    git clone https://github.com/aws-robotics/cloudwatchmetrics-ros2.git -b release-latest
    cd  ~/ros-workspace 
    apt-get update && rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the cloudwatchmetrics-ros2 app
    ```
    source  /opt/ros/dashing/setup.bash
    cd ~/ros-workspace && colcon build
    source ~/ros-workspace/install/local_setup.bash
    colcon test && colcon test-result --all
    export HOME=/root
    ```

4. Execute the cloudwatchmetrics-ros2 app

    i. Being in the same terminal and directory, enter the command 
    ```
    ros2 launch cloudwatch_metrics_collector cloudwatch_metrics_collector.launch.py
    ```
    ii. Open another terminal and send text request message
    ```
    source  /opt/ros/dashing/setup.bash
    timestamp=$(date +%s);
    ros2 topic pub /metrics ros_monitoring_msgs/msg/MetricList"{metrics: [{header:{stamp:{sec: ${timestamp}, nanosec: 0}} , metric_name: 'cw_offline_metric', unit: 'Count', value: 1.0, time_stamp: {sec: ${timestamp}, nanosec: 0}, dimensions: [{name: 'RB5_device_dimension', value: 'RB5_device_value'}]}]}"
    ```

    After executing these commands login to AWS Cloudwatch Metrics Dashboard to verify metrics.  
    https://us-west-2.console.aws.amazon.com/cloudwatch/home?region=us-west-2#metricsV2: 
    
5. Cloudwatchmetrics-ros2 application execution outputs

    ![Terminal](image/CloudwatchMetrics_Screenshot_1.PNG)
    [Alt tag: ”Node launch”]

    ![Terminal](image/CloudwatchMetrics_Screenshot_2.PNG)
    [Alt tag: ”Sending test metric”]
    
    ![AWS Dashboard](image/CloudwatchMetrics_Screenshot_3.PNG)
    [Alt tag: ”AWS Cloudwatch Metric Dashboard showing sent test metric”]

    ![AWS Dashboard](image/CloudwatchMetrics_Screenshot_4.PNG)
    [Alt tag: ”AWS Cloudwatch Metric Dashboard showing sent test metric”]
    
For more information regarding this please refer below link:
https://github.com/aws-robotics/cloudwatchmetrics-ros2





