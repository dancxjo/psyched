pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Goals() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__Goals__init(msg: *mut Goals) -> bool;
    fn nav_msgs__msg__Goals__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Goals>, size: usize) -> bool;
    fn nav_msgs__msg__Goals__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Goals>);
    fn nav_msgs__msg__Goals__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Goals>, out_seq: *mut rosidl_runtime_rs::Sequence<Goals>) -> bool;
}

// Corresponds to nav_msgs__msg__Goals
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Goals {
    pub header: std_msgs::msg::rmw::Header,
    pub goals: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
}



impl Default for Goals {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__Goals__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__Goals__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Goals {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Goals__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Goals__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Goals__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Goals {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Goals where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/Goals";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Goals() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__GridCells() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__GridCells__init(msg: *mut GridCells) -> bool;
    fn nav_msgs__msg__GridCells__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GridCells>, size: usize) -> bool;
    fn nav_msgs__msg__GridCells__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GridCells>);
    fn nav_msgs__msg__GridCells__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GridCells>, out_seq: *mut rosidl_runtime_rs::Sequence<GridCells>) -> bool;
}

// Corresponds to nav_msgs__msg__GridCells
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GridCells {
    pub header: std_msgs::msg::rmw::Header,
    pub cell_width: f32,
    pub cell_height: f32,
    pub cells: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,
}



impl Default for GridCells {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__GridCells__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__GridCells__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GridCells {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__GridCells__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__GridCells__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__GridCells__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GridCells {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GridCells where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/GridCells";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__GridCells() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__MapMetaData() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__MapMetaData__init(msg: *mut MapMetaData) -> bool;
    fn nav_msgs__msg__MapMetaData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MapMetaData>, size: usize) -> bool;
    fn nav_msgs__msg__MapMetaData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MapMetaData>);
    fn nav_msgs__msg__MapMetaData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MapMetaData>, out_seq: *mut rosidl_runtime_rs::Sequence<MapMetaData>) -> bool;
}

// Corresponds to nav_msgs__msg__MapMetaData
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapMetaData {
    pub map_load_time: builtin_interfaces::msg::rmw::Time,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: geometry_msgs::msg::rmw::Pose,
}



impl Default for MapMetaData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__MapMetaData__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__MapMetaData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MapMetaData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__MapMetaData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__MapMetaData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__MapMetaData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MapMetaData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MapMetaData where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/MapMetaData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__MapMetaData() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__OccupancyGrid() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__OccupancyGrid__init(msg: *mut OccupancyGrid) -> bool;
    fn nav_msgs__msg__OccupancyGrid__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<OccupancyGrid>, size: usize) -> bool;
    fn nav_msgs__msg__OccupancyGrid__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<OccupancyGrid>);
    fn nav_msgs__msg__OccupancyGrid__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<OccupancyGrid>, out_seq: *mut rosidl_runtime_rs::Sequence<OccupancyGrid>) -> bool;
}

// Corresponds to nav_msgs__msg__OccupancyGrid
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OccupancyGrid {
    pub header: std_msgs::msg::rmw::Header,
    pub info: crate::msg::rmw::MapMetaData,
    pub data: rosidl_runtime_rs::Sequence<i8>,
}



impl Default for OccupancyGrid {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__OccupancyGrid__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__OccupancyGrid__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for OccupancyGrid {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__OccupancyGrid__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__OccupancyGrid__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__OccupancyGrid__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for OccupancyGrid {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for OccupancyGrid where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/OccupancyGrid";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__OccupancyGrid() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Odometry() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__Odometry__init(msg: *mut Odometry) -> bool;
    fn nav_msgs__msg__Odometry__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Odometry>, size: usize) -> bool;
    fn nav_msgs__msg__Odometry__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Odometry>);
    fn nav_msgs__msg__Odometry__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Odometry>, out_seq: *mut rosidl_runtime_rs::Sequence<Odometry>) -> bool;
}

// Corresponds to nav_msgs__msg__Odometry
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Odometry {
    pub header: std_msgs::msg::rmw::Header,
    pub child_frame_id: rosidl_runtime_rs::String,
    pub pose: geometry_msgs::msg::rmw::PoseWithCovariance,
    pub twist: geometry_msgs::msg::rmw::TwistWithCovariance,
}



impl Default for Odometry {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__Odometry__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__Odometry__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Odometry {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Odometry__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Odometry__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Odometry__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Odometry {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Odometry where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/Odometry";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Odometry() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__msg__Path__init(msg: *mut Path) -> bool;
    fn nav_msgs__msg__Path__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Path>, size: usize) -> bool;
    fn nav_msgs__msg__Path__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Path>);
    fn nav_msgs__msg__Path__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Path>, out_seq: *mut rosidl_runtime_rs::Sequence<Path>) -> bool;
}

// Corresponds to nav_msgs__msg__Path
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Path {
    pub header: std_msgs::msg::rmw::Header,
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,
}



impl Default for Path {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__msg__Path__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__msg__Path__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Path {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Path__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Path__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__msg__Path__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Path {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Path where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/msg/Path";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Goals {
    pub header: std_msgs::msg::Header,
    pub goals: Vec<geometry_msgs::msg::PoseStamped>,
}



impl Default for Goals {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Goals::default())
  }
}

impl rosidl_runtime_rs::Message for Goals {
  type RmwMsg = crate::msg::rmw::Goals;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        goals: msg.goals
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        goals: msg.goals
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      goals: msg.goals
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GridCells {
    pub header: std_msgs::msg::Header,
    pub cell_width: f32,
    pub cell_height: f32,
    pub cells: Vec<geometry_msgs::msg::Point>,
}



impl Default for GridCells {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::GridCells::default())
  }
}

impl rosidl_runtime_rs::Message for GridCells {
  type RmwMsg = crate::msg::rmw::GridCells;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        cell_width: msg.cell_width,
        cell_height: msg.cell_height,
        cells: msg.cells
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      cell_width: msg.cell_width,
      cell_height: msg.cell_height,
        cells: msg.cells
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      cell_width: msg.cell_width,
      cell_height: msg.cell_height,
      cells: msg.cells
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MapMetaData {
    pub map_load_time: builtin_interfaces::msg::Time,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: geometry_msgs::msg::Pose,
}



impl Default for MapMetaData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MapMetaData::default())
  }
}

impl rosidl_runtime_rs::Message for MapMetaData {
  type RmwMsg = crate::msg::rmw::MapMetaData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_load_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.map_load_time)).into_owned(),
        resolution: msg.resolution,
        width: msg.width,
        height: msg.height,
        origin: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.origin)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_load_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map_load_time)).into_owned(),
      resolution: msg.resolution,
      width: msg.width,
      height: msg.height,
        origin: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.origin)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_load_time: builtin_interfaces::msg::Time::from_rmw_message(msg.map_load_time),
      resolution: msg.resolution,
      width: msg.width,
      height: msg.height,
      origin: geometry_msgs::msg::Pose::from_rmw_message(msg.origin),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct OccupancyGrid {
    pub header: std_msgs::msg::Header,
    pub info: crate::msg::MapMetaData,
    pub data: Vec<i8>,
}



impl Default for OccupancyGrid {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::OccupancyGrid::default())
  }
}

impl rosidl_runtime_rs::Message for OccupancyGrid {
  type RmwMsg = crate::msg::rmw::OccupancyGrid;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        info: crate::msg::MapMetaData::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        info: crate::msg::MapMetaData::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      info: crate::msg::MapMetaData::from_rmw_message(msg.info),
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Odometry {
    pub header: std_msgs::msg::Header,
    pub child_frame_id: std::string::String,
    pub pose: geometry_msgs::msg::PoseWithCovariance,
    pub twist: geometry_msgs::msg::TwistWithCovariance,
}



impl Default for Odometry {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Odometry::default())
  }
}

impl rosidl_runtime_rs::Message for Odometry {
  type RmwMsg = crate::msg::rmw::Odometry;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        child_frame_id: msg.child_frame_id.as_str().into(),
        pose: geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        twist: geometry_msgs::msg::TwistWithCovariance::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        child_frame_id: msg.child_frame_id.as_str().into(),
        pose: geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        twist: geometry_msgs::msg::TwistWithCovariance::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      child_frame_id: msg.child_frame_id.to_string(),
      pose: geometry_msgs::msg::PoseWithCovariance::from_rmw_message(msg.pose),
      twist: geometry_msgs::msg::TwistWithCovariance::from_rmw_message(msg.twist),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Path {
    pub header: std_msgs::msg::Header,
    pub poses: Vec<geometry_msgs::msg::PoseStamped>,
}



impl Default for Path {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Path::default())
  }
}

impl rosidl_runtime_rs::Message for Path {
  type RmwMsg = crate::msg::rmw::Path;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
  }
}


