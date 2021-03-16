### Prerequisites to AWS-Sample Application

Make sure to be ready with the prerequisites listed here:

1. Bring up the Qualcomm Robotics RB5 and install ROS2 following the instructions available here: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up
2. Install AWS CLI (version - 2.0.30)  following the instructions from https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2-linux.html
3. Set up role and get Access keys through AWS CLI
AWS CLI is a tool that enables you to interact with AWS services through commands in your command-line shell.

    i.From a host PC, Go to https://console.aws.amazon.com/iam/home#/home and select “Users” from the left tab and choose “Add User”
    ![AWS](image/AWS-CLI-1.png)
    
     ![AWS](image/AWS-CLI-2.png)
     
    ii.Enter Username, choose “Programmatic Access” and click on Next: Permissions
    
    ![AWS](image/AWS-CLI-3.png)
    
    iii. In the 3 next pages, choose Next: Tags, Next: Review and Create User. You can see that the user is created as below
    
    ![AWS](image/AWS-CLI-4.png)
    
    iv. Choose Download .csv file and make sure to store the keys in a secure location, as you will not have access to the secret access key again after this dialog box closes
    Your credentials look similar to this:
    Access key ID: _AKIAIOSFODNN7EXAMPLE_
    Secret access key: _wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY_
    
    v. Configure AWS CLI in Qualcomm Robotics RB5
    - aws configure command is the fastest way to set up AWS CLI installation.
Open a terminal in Qualcomm Robotics RB5 and enter the command below
    ```
    $ aws configure 
    AWS Access Key ID [None]: AKIAIOSFODNN7EXAMPLE 
    AWS Secret Access Key [None]: wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY 
    Default region name [None]: us-west-2-example 
    Default output format [None]: json
    ```
    
To be able to use AWS Services,  permissions for the respective services should be setup on the AWS Dashboard.

For more information on colcon bundle and bundle installation please read:
https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2-linux.html



