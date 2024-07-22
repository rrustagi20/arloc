//****************************************************************** 
//
// Get estimated tag pose in the camera frame.
//
// Note on frames:
// The raw AprilTag 2 uses the following frames:
//
//   - camera /tag frame: looking from behind the camera (like a
//     photographer), x is right, y is up and z is towards you           
//     (i.e. the back of camera)
//
//                ______ X
//               |
//               |          CAMERA FRAME
//               |
//                 Y
//
//   - tag / camera frame: looking straight at the tag (oriented correctly),
//     x is right, y is down and z is away from you (into the tag).
//
//                ______ X
//               |
//               |          TAG FRAME
//               |
//                 Y
//
//   - tag frame: looking straight at the tag (oriented correctly),
//     x is right, y is down and z is away from you (into the tag).
//
//                Z
//               |
//               |          ROBOT FRAME
//       Y_______|
//                
// But we want:
//   - camera frame: looking from behind the camera (like a
//     photographer), x is right, y is down and z is straight
//     ahead
//   - tag frame: looking straight at the tag (oriented correctly),
//     x is right, y is up and z is towards you (out of the tag).
// Using these frames together with cv::solvePnP directly avoids
// AprilTag 2's frames altogether.
//
//****************************************************************** 