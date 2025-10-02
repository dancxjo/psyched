pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "trajectory_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__JointTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "trajectory_msgs__rosidl_generator_c")]
extern "C" {
    fn trajectory_msgs__msg__JointTrajectory__init(msg: *mut JointTrajectory) -> bool;
    fn trajectory_msgs__msg__JointTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointTrajectory>, size: usize) -> bool;
    fn trajectory_msgs__msg__JointTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointTrajectory>);
    fn trajectory_msgs__msg__JointTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<JointTrajectory>) -> bool;
}

// Corresponds to trajectory_msgs__msg__JointTrajectory
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointTrajectory {
    pub header: std_msgs::msg::rmw::Header,
    pub joint_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub points: rosidl_runtime_rs::Sequence<crate::msg::rmw::JointTrajectoryPoint>,
}



impl Default for JointTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !trajectory_msgs__msg__JointTrajectory__init(&mut msg as *mut _) {
        panic!("Call to trajectory_msgs__msg__JointTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "trajectory_msgs/msg/JointTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__JointTrajectory() }
  }
}


#[link(name = "trajectory_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__JointTrajectoryPoint() -> *const std::ffi::c_void;
}

#[link(name = "trajectory_msgs__rosidl_generator_c")]
extern "C" {
    fn trajectory_msgs__msg__JointTrajectoryPoint__init(msg: *mut JointTrajectoryPoint) -> bool;
    fn trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointTrajectoryPoint>, size: usize) -> bool;
    fn trajectory_msgs__msg__JointTrajectoryPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointTrajectoryPoint>);
    fn trajectory_msgs__msg__JointTrajectoryPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointTrajectoryPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<JointTrajectoryPoint>) -> bool;
}

// Corresponds to trajectory_msgs__msg__JointTrajectoryPoint
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointTrajectoryPoint {
    pub positions: rosidl_runtime_rs::Sequence<f64>,
    pub velocities: rosidl_runtime_rs::Sequence<f64>,
    pub accelerations: rosidl_runtime_rs::Sequence<f64>,
    pub effort: rosidl_runtime_rs::Sequence<f64>,
    pub time_from_start: builtin_interfaces::msg::rmw::Duration,
}



impl Default for JointTrajectoryPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !trajectory_msgs__msg__JointTrajectoryPoint__init(&mut msg as *mut _) {
        panic!("Call to trajectory_msgs__msg__JointTrajectoryPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointTrajectoryPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectoryPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__JointTrajectoryPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointTrajectoryPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointTrajectoryPoint where Self: Sized {
  const TYPE_NAME: &'static str = "trajectory_msgs/msg/JointTrajectoryPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__JointTrajectoryPoint() }
  }
}


#[link(name = "trajectory_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__MultiDOFJointTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "trajectory_msgs__rosidl_generator_c")]
extern "C" {
    fn trajectory_msgs__msg__MultiDOFJointTrajectory__init(msg: *mut MultiDOFJointTrajectory) -> bool;
    fn trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectory>, size: usize) -> bool;
    fn trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectory>);
    fn trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiDOFJointTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectory>) -> bool;
}

// Corresponds to trajectory_msgs__msg__MultiDOFJointTrajectory
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointTrajectory {
    pub header: std_msgs::msg::rmw::Header,
    pub joint_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub points: rosidl_runtime_rs::Sequence<crate::msg::rmw::MultiDOFJointTrajectoryPoint>,
}



impl Default for MultiDOFJointTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !trajectory_msgs__msg__MultiDOFJointTrajectory__init(&mut msg as *mut _) {
        panic!("Call to trajectory_msgs__msg__MultiDOFJointTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiDOFJointTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiDOFJointTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "trajectory_msgs/msg/MultiDOFJointTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__MultiDOFJointTrajectory() }
  }
}


#[link(name = "trajectory_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint() -> *const std::ffi::c_void;
}

#[link(name = "trajectory_msgs__rosidl_generator_c")]
extern "C" {
    fn trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__init(msg: *mut MultiDOFJointTrajectoryPoint) -> bool;
    fn trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectoryPoint>, size: usize) -> bool;
    fn trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectoryPoint>);
    fn trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiDOFJointTrajectoryPoint>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointTrajectoryPoint>) -> bool;
}

// Corresponds to trajectory_msgs__msg__MultiDOFJointTrajectoryPoint
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointTrajectoryPoint {
    pub transforms: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Transform>,
    pub velocities: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Twist>,
    pub accelerations: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Twist>,
    pub time_from_start: builtin_interfaces::msg::rmw::Duration,
}



impl Default for MultiDOFJointTrajectoryPoint {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__init(&mut msg as *mut _) {
        panic!("Call to trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiDOFJointTrajectoryPoint {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointTrajectoryPoint {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiDOFJointTrajectoryPoint where Self: Sized {
  const TYPE_NAME: &'static str = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointTrajectory {
    pub header: std_msgs::msg::Header,
    pub joint_names: Vec<std::string::String>,
    pub points: Vec<crate::msg::JointTrajectoryPoint>,
}



impl Default for JointTrajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JointTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for JointTrajectory {
  type RmwMsg = crate::msg::rmw::JointTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        points: msg.points
          .into_iter()
          .map(|elem| crate::msg::JointTrajectoryPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        joint_names: msg.joint_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        points: msg.points
          .iter()
          .map(|elem| crate::msg::JointTrajectoryPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      points: msg.points
          .into_iter()
          .map(crate::msg::JointTrajectoryPoint::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointTrajectoryPoint {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub accelerations: Vec<f64>,
    pub effort: Vec<f64>,
    pub time_from_start: builtin_interfaces::msg::Duration,
}



impl Default for JointTrajectoryPoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JointTrajectoryPoint::default())
  }
}

impl rosidl_runtime_rs::Message for JointTrajectoryPoint {
  type RmwMsg = crate::msg::rmw::JointTrajectoryPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        positions: msg.positions.into(),
        velocities: msg.velocities.into(),
        accelerations: msg.accelerations.into(),
        effort: msg.effort.into(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_from_start)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        positions: msg.positions.as_slice().into(),
        velocities: msg.velocities.as_slice().into(),
        accelerations: msg.accelerations.as_slice().into(),
        effort: msg.effort.as_slice().into(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_from_start)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      positions: msg.positions
          .into_iter()
          .collect(),
      velocities: msg.velocities
          .into_iter()
          .collect(),
      accelerations: msg.accelerations
          .into_iter()
          .collect(),
      effort: msg.effort
          .into_iter()
          .collect(),
      time_from_start: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_from_start),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointTrajectory {
    pub header: std_msgs::msg::Header,
    pub joint_names: Vec<std::string::String>,
    pub points: Vec<crate::msg::MultiDOFJointTrajectoryPoint>,
}



impl Default for MultiDOFJointTrajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MultiDOFJointTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointTrajectory {
  type RmwMsg = crate::msg::rmw::MultiDOFJointTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        points: msg.points
          .into_iter()
          .map(|elem| crate::msg::MultiDOFJointTrajectoryPoint::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        joint_names: msg.joint_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        points: msg.points
          .iter()
          .map(|elem| crate::msg::MultiDOFJointTrajectoryPoint::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      points: msg.points
          .into_iter()
          .map(crate::msg::MultiDOFJointTrajectoryPoint::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointTrajectoryPoint {
    pub transforms: Vec<geometry_msgs::msg::Transform>,
    pub velocities: Vec<geometry_msgs::msg::Twist>,
    pub accelerations: Vec<geometry_msgs::msg::Twist>,
    pub time_from_start: builtin_interfaces::msg::Duration,
}



impl Default for MultiDOFJointTrajectoryPoint {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MultiDOFJointTrajectoryPoint::default())
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointTrajectoryPoint {
  type RmwMsg = crate::msg::rmw::MultiDOFJointTrajectoryPoint;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        transforms: msg.transforms
          .into_iter()
          .map(|elem| geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        velocities: msg.velocities
          .into_iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        accelerations: msg.accelerations
          .into_iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Owned(msg.time_from_start)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        transforms: msg.transforms
          .iter()
          .map(|elem| geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        velocities: msg.velocities
          .iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        accelerations: msg.accelerations
          .iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        time_from_start: builtin_interfaces::msg::Duration::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_from_start)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      transforms: msg.transforms
          .into_iter()
          .map(geometry_msgs::msg::Transform::from_rmw_message)
          .collect(),
      velocities: msg.velocities
          .into_iter()
          .map(geometry_msgs::msg::Twist::from_rmw_message)
          .collect(),
      accelerations: msg.accelerations
          .into_iter()
          .map(geometry_msgs::msg::Twist::from_rmw_message)
          .collect(),
      time_from_start: builtin_interfaces::msg::Duration::from_rmw_message(msg.time_from_start),
    }
  }
}


