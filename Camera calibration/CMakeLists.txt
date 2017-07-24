cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
project( DisplayImage )
find_package( OpenCV REQUIRED )
add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )


add_executable( DrawAruco DrawAruco.cpp )
target_link_libraries( DrawAruco ${OpenCV_LIBS} )

add_executable( FindAruco FindAruco.cpp )
target_link_libraries( FindAruco ${OpenCV_LIBS} )

add_executable( CalibrateCamera CalibrateCamera.cpp )
target_link_libraries( CalibrateCamera ${OpenCV_LIBS} )

add_executable( CalibrateCharuco calibrate_camera_charuco.cpp )
target_link_libraries( CalibrateCharuco ${OpenCV_LIBS} )

add_executable( DrawArucoBoard create_board.cpp )
target_link_libraries( DrawArucoBoard ${OpenCV_LIBS} )

add_executable( DrawCharuco create_board_charuco.cpp )
target_link_libraries( DrawCharuco ${OpenCV_LIBS} )