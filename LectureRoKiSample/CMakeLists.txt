
if(NOT BUILD_ROKI_PLUGIN)
  return()
endif()

## Arm sample
add_cnoid_simple_controller(Arm2dofPushController Arm2dofPushController.cpp)

configure_file(arm_2dof_geared.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(arm_2dof_geared.yaml ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(arm_2dof_notgeared.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

configure_file(LectureRoKiArmGeared.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(LectureRoKiArmTrq.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(LectureRoKiArmNone.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

## SR1 walk sample on hard and soft ground
add_cnoid_simple_controller(SR1WalkStopPatternController SR1WalkStopPatternController.cpp)

configure_file(floor_hard.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(floor_soft.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(contact_info.zci ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

configure_file(LectureRoKiSoftHardGround.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

## break wall sample
add_cnoid_simple_controller(Arm2dofBreakWallController Arm2dofBreakWallController.cpp)

configure_file(breakWallModel.wrl ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(breakWall_hard.yaml ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(breakWall_soft.yaml ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)
configure_file(breakWall_none.yaml ${CNOID_SOURCE_SHARE_DIR}/model/misc COPYONLY)

configure_file(LectureRoKiBreakWallHard.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(LectureRoKiBreakWallSoft.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
configure_file(LectureRoKiBreakWallNone.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)
