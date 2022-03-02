# Advanced Control Systems
### Simone Cremasco VR468971

## Usage
First run `main_3DoF_robots.m`, then you can run any of the `assignment#.m` you can find inside assignments folder.

## How it works
Two classes have been defined: `MyRobot` and 'Link'.

`main_3DoF_robots.m` will initialize a variable of the class myRobot which will pre-compute in a symbolic way most of the matrices required.

Some useful methods have been defined in MyRobot class:
- `show` will show the robot in it's current configuration
- `setSingleConfig` to change the value of a joint
- `setAllConfig` to set all joint values
- `setValues`. This one is used to solve the symbolic variable passed as param. It will replace all symbolic variables like masses, length links and so on.

### Report
The report can be found at https://www.overleaf.com/read/pbhdtdkxhcnv
