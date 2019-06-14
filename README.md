# PhaseSpace
## Launch PhaseSpace
### Preparation
* Connect the robot network to the Internet (we need Internet access to update the time on each computer).
* Operating the robot onboard PC from the demo PC via SSH
  * `sshyuri` with password `mit`.
  * `updatetime`
    * Sync the time in the robot onboard PC with the Internet.
* Operating the demo PC directly
  * Change `.bashrc` properly.
    * Please don't source multiple `setup.bash` files in one terminal.
      * For example, when the terminal has already sourced a file `a`, and you want to source another one `b`, please **kill** that terminal and create a new terminal that only sources `b` rather than `a`.
  * `updatetime`

### On the demo PC, start to run these commands:
* Terminal 1
  * Tab 1
    * `sshyuri` with password `mit`.
    * `roscore`
  * Tab 2
    * `sshyuri` with password `mit`.
    * `roslaunch phasespace_acquisition phasespace_publisher.launch`
      * Note that if you are running `despot.launch` before running `phasespace_publisher.launch`, `phasespace_publisher.launch` might fail due to communication problems.
      * After you physically connect the robot with PhaseSpace, SSH into the robot router might take forever. Therefore, you should open all the necessary terminals with SSH **before** connecting to PhaseSpace.
      * If the connection keeps failing, then you can try reconnect the Ethernet cable physically or re-Enable networking in Ubuntu.
        * Sometimes when you switch the connection of the robot network from the Internet to PhaseSpace, `phasespace_publisher.launch` might fail and you might have to try re-connecting the Ethernet cables physically or restarting the Ubuntu network manager.
          * In order to avoid this trouble, please first disconnect physically the Ethernet connection between the robot system and the Internet, shut down Ubuntu network, wait 10 seconds, reconnect the Ethernet cable between the robot system and PhaseSpace. Usually in this way, `phasespace_publisher.launch` will work in the first attempt.
  * Tab 3
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition msg_2_tf.py`
      * This will convert the LED poses associated with the left palm, right palm, and elbow to TF
      * `rosrun phasespace_acquisition msg_2_tf_avg_palm.py`

## Robot base localization
* In the PhaseSpace environment, we don't have the markers to track the robot base pose in real time. Therefore, we need to hard code the robot base pose.
  * The limitation of this is that the robot base cannot move.
* We have pre-defined the pose of the robot base - `/odom_combined` in the frame of `map` and saved it as `map_2_odomCombined_trans` in `./config/fake_localization.yaml`.
* In real time, we will run this following command to keep publishing the robot base pose to TF.
  * `rosrun phasespace_acquisition fake_localization.py`

### Calibrating the pre-defined robot base pose
## Preparation
* Operating the robot onboard PC from the demo PC via SSH
  * `sshyuri` with password `mit`.
  * `updatetime`
    * Sync the time in the robot onboard PC with the Internet.
* Operating the demo PC directly
  * Change `.bashrc`:
    * Uncomment the line of `source .../setup.bash` with `vaibhav`.
    * Comment other lines of `source .../setup.bash`.
    * Please don't source multiple `setup.bash` files in one terminal.
      * For example, when the terminal has already sourced a file `a`, and you want to source another one `b`, please **kill** that terminal and create a new terminal that only sources `b` rather than `a`.
  * `updatetime`

## On the demo PC, start to run these commands:
### 1st rough calibration
* Terminal 1
  * Tab 1
    * `sshyuri` with password `mit`.
    * `roscore`
  * Tab 2
    * `sshyuri` with password `mit`.
    * `roslaunch yuri_launch despot.launch`
  * Tab 4
    * `sshyuri` with password `mit`.
    * `roslaunch phasespace_acquisition phasespace_publisher.launch`
  * Tab 5
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition msg_2_tf.py`
  * Tab 5
    * `rosrun phasespace_acquisition localization_calibration.py`
      * This will print out the translation from `map` to the PhaseSpace marker named as `PHASESPACE_ID_FOR_CALIBRATION` in `localization_calibration.py`.
      * You can place that LED onto the small red taped dot on the robot base, which is `odom_combined`.
      * Then you can read the translation printed by `localization_calibration.py` and put the translation in `./config/fake_localization.yaml` as `map_2_odomCombined_trans`.
      * Note that since usually we cannot place the LED glove under the robot base, we have to place it on the top of the robot base during calibration. So the translation of the LED represents the center of the top of the robot base. However, the `odom_combined` is the center of the bottom of the robot base. Therefore, here you have to manually make the last element of the translation (`z`) to be `0`.
      * For orientation, you have to try the euler angles by yourself. Usually you can align the robot so that you only need to rotate `90` or `180`

### 2nd fine calibration
* Terminal 1
  * Tab 1
    * `sshyuri` with password `mit`.
    * `roscore`
  * Tab 2
    * `sshyuri` with password `mit`.
    * `roslaunch yuri_launch despot.launch`
  * Tab 3
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition fake_localization.py`
  * Tab 4
    * `sshyuri` with password `mit`.
    * `roslaunch phasespace_acquisition phasespace_publisher.launch`
  * Tab 5
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition msg_2_tf.py`
* Terminal 2
  * Tab 1
    * `rosyuri`
      * Connect the demo PC with the `roscore` running on the robot onboard PC.
    * `roslaunch cob_navigation_global rviz.launch`
      * Verifying whether `fake_localization` through PhaseSpace or VICON is working.
    * `roslaunch yuri_launch despot_rviz.launch`
      * Move the robot to a pose and place the LED associated with the frame `phasespace/albow` onto a position that is recognizable on RViz, such as the center of the letter `R` of the robot elbow.
      * On RViz, use the `TopDownOrtho` view. Look at the TF of the frame `phasespace/albow`. Tweak the `map_2_odomCombined_trans` until the real and RViz look the same.

# Finding out the translation of an object
* Terminal 1
  * Tab 1
    * `sshyuri` with password `mit`.
    * `roscore`
  * Tab 2
    * `sshyuri` with password `mit`.
    * `roslaunch yuri_launch despot.launch`
  * Tab 3
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition fake_localization.py`
  * Tab 4
    * `sshyuri` with password `mit`.
    * `roslaunch phasespace_acquisition phasespace_publisher.launch`
  * Tab 5
    * `sshyuri` with password `mit`.
    * `rosrun phasespace_acquisition msg_2_tf.py`
  * Tab 6
    * `rosrun phasespace_acquisition object_translation_calibration.py`
      * This will print the TF from the `BASE_FRAME` (usually `/odom_combined`) to `PHASESPACE_ID_FOR_CALIBRATION` which are both defined in `object_translation_calibration.py`.