^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2022-09-29)
------------------
* [jackal_viz] Changes rviz to rviz2.
* Contributors: Tony Baltovski

1.0.0 (2022-09-29)
------------------
* View diagnostics with rqt_robot_monitor
* Fixed jackal_desktop metapackage
  Added maintainers
* Added rviz configurations for slam and nav2
  Updated launch files
  Removed ROS1 files
* ROS 2 Port
* Contributors: David V. Lu, Roni Kreinin

0.4.1 (2022-01-16)
------------------
* Updated jackal_viz (`#3 <https://github.com/jackal/jackal_desktop/issues/3>`_)
  * Add rqt_gui as run_depend
  * Add rqt directory and launch check
* Diagnostic Viewer (`#2 <https://github.com/jackal/jackal_desktop/issues/2>`_)
  * Added 'view_diagnostics.launch' to start rqt with console and robot monitor
  * Added noetic .perspective, now are dependant on ROS_DISTRO
  * Removed distro specific perspectives
* Contributors: luis-camero

0.4.0 (2020-04-21)
------------------
* [jackal_viz] Removed joint_state_publisher since joint_state_publisher_gui is generating the same data.
* Fix a deprecation warning with the joint state publisher gui
* Contributors: Chris Iverach-Brereton, Tony Baltovski

0.3.2 (2016-06-06)
------------------
* Changed laser scan topics to /front/scan.
* Contributors: Tony Baltovski

0.3.1 (2015-01-20)
------------------
* Add version number blocks.
* Switch view_model to use optenv-style URDF parameterization.
* Contributors: Mike Purvis

0.3.0 (2014-12-12)
------------------
* Add camera argument to view model
* Add rviz configuration for navigation demo.
* Contributors: Shokoofeh Pourmehr, spourmehr

0.2.0 (2014-09-10)
------------------
* Support front_laser arg in view_model.launch.
* Add view_robot launcher for marker teleop.
* Contributors: Mike Purvis

0.1.1 (2014-09-06)
------------------
* Fix install target for jackal_viz.
* Contributors: Mike Purvis

0.1.0 (2014-09-05)
------------------
* Initial release.
* Contributors: Mike Purvis
