

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetMap_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for GetMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetMap_Request {
  type RmwMsg = crate::srv::rmw::GetMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetMap_Response {
    pub map: crate::msg::OccupancyGrid,
}



impl Default for GetMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetMap_Response {
  type RmwMsg = crate::srv::rmw::GetMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: crate::msg::OccupancyGrid::from_rmw_message(msg.map),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPlan_Request {
    pub start: geometry_msgs::msg::PoseStamped,
    pub goal: geometry_msgs::msg::PoseStamped,
    pub tolerance: f32,
}



impl Default for GetPlan_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetPlan_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetPlan_Request {
  type RmwMsg = crate::srv::rmw::GetPlan_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
        tolerance: msg.tolerance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      tolerance: msg.tolerance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
      tolerance: msg.tolerance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPlan_Response {
    pub plan: crate::msg::Path,
}



impl Default for GetPlan_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::GetPlan_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetPlan_Response {
  type RmwMsg = crate::srv::rmw::GetPlan_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        plan: crate::msg::Path::into_rmw_message(std::borrow::Cow::Owned(msg.plan)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        plan: crate::msg::Path::into_rmw_message(std::borrow::Cow::Borrowed(&msg.plan)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      plan: crate::msg::Path::from_rmw_message(msg.plan),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {
    pub map_url: std::string::String,
}



impl Default for LoadMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::LoadMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = crate::srv::rmw::LoadMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_url: msg.map_url.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_url: msg.map_url.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    pub map: crate::msg::OccupancyGrid,
    pub result: u8,
}

impl LoadMap_Response {
    pub const RESULT_SUCCESS: u8 = 0;
    pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
    pub const RESULT_INVALID_MAP_DATA: u8 = 2;
    pub const RESULT_INVALID_MAP_METADATA: u8 = 3;
    pub const RESULT_UNDEFINED_FAILURE: u8 = 255;
}


impl Default for LoadMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::LoadMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = crate::srv::rmw::LoadMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
        result: msg.result,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
      result: msg.result,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: crate::msg::OccupancyGrid::from_rmw_message(msg.map),
      result: msg.result,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetMap_Request {
    pub map: crate::msg::OccupancyGrid,
    pub initial_pose: geometry_msgs::msg::PoseWithCovarianceStamped,
}



impl Default for SetMap_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetMap_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetMap_Request {
  type RmwMsg = crate::srv::rmw::SetMap_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Owned(msg.map)).into_owned(),
        initial_pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.initial_pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map: crate::msg::OccupancyGrid::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map)).into_owned(),
        initial_pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.initial_pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map: crate::msg::OccupancyGrid::from_rmw_message(msg.map),
      initial_pose: geometry_msgs::msg::PoseWithCovarianceStamped::from_rmw_message(msg.initial_pose),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetMap_Response {
    pub success: bool,
}



impl Default for SetMap_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetMap_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetMap_Response {
  type RmwMsg = crate::srv::rmw::SetMap_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}






#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetMap() -> *const std::ffi::c_void;
}

// Corresponds to nav_msgs__srv__GetMap
pub struct GetMap;

impl rosidl_runtime_rs::Service for GetMap {
  type Request = crate::srv::GetMap_Request;
  type Response = crate::srv::GetMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetMap() }
  }
}




#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetPlan() -> *const std::ffi::c_void;
}

// Corresponds to nav_msgs__srv__GetPlan
pub struct GetPlan;

impl rosidl_runtime_rs::Service for GetPlan {
  type Request = crate::srv::GetPlan_Request;
  type Response = crate::srv::GetPlan_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetPlan() }
  }
}




#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__LoadMap() -> *const std::ffi::c_void;
}

// Corresponds to nav_msgs__srv__LoadMap
pub struct LoadMap;

impl rosidl_runtime_rs::Service for LoadMap {
  type Request = crate::srv::LoadMap_Request;
  type Response = crate::srv::LoadMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__LoadMap() }
  }
}




#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__SetMap() -> *const std::ffi::c_void;
}

// Corresponds to nav_msgs__srv__SetMap
pub struct SetMap;

impl rosidl_runtime_rs::Service for SetMap {
  type Request = crate::srv::SetMap_Request;
  type Response = crate::srv::SetMap_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__SetMap() }
  }
}




pub mod rmw {

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__GetMap_Request__init(msg: *mut GetMap_Request) -> bool;
    fn nav_msgs__srv__GetMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetMap_Request>, size: usize) -> bool;
    fn nav_msgs__srv__GetMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetMap_Request>);
    fn nav_msgs__srv__GetMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetMap_Request>) -> bool;
}

// Corresponds to nav_msgs__srv__GetMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetMap_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for GetMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__GetMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__GetMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/GetMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetMap_Request() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__GetMap_Response__init(msg: *mut GetMap_Response) -> bool;
    fn nav_msgs__srv__GetMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetMap_Response>, size: usize) -> bool;
    fn nav_msgs__srv__GetMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetMap_Response>);
    fn nav_msgs__srv__GetMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetMap_Response>) -> bool;
}

// Corresponds to nav_msgs__srv__GetMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetMap_Response {
    pub map: crate::msg::rmw::OccupancyGrid,
}



impl Default for GetMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__GetMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__GetMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/GetMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetMap_Response() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetPlan_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__GetPlan_Request__init(msg: *mut GetPlan_Request) -> bool;
    fn nav_msgs__srv__GetPlan_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Request>, size: usize) -> bool;
    fn nav_msgs__srv__GetPlan_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Request>);
    fn nav_msgs__srv__GetPlan_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPlan_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Request>) -> bool;
}

// Corresponds to nav_msgs__srv__GetPlan_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPlan_Request {
    pub start: geometry_msgs::msg::rmw::PoseStamped,
    pub goal: geometry_msgs::msg::rmw::PoseStamped,
    pub tolerance: f32,
}



impl Default for GetPlan_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__GetPlan_Request__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__GetPlan_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPlan_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPlan_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPlan_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/GetPlan_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetPlan_Request() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetPlan_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__GetPlan_Response__init(msg: *mut GetPlan_Response) -> bool;
    fn nav_msgs__srv__GetPlan_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Response>, size: usize) -> bool;
    fn nav_msgs__srv__GetPlan_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Response>);
    fn nav_msgs__srv__GetPlan_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPlan_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPlan_Response>) -> bool;
}

// Corresponds to nav_msgs__srv__GetPlan_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPlan_Response {
    pub plan: crate::msg::rmw::Path,
}



impl Default for GetPlan_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__GetPlan_Response__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__GetPlan_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPlan_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__GetPlan_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPlan_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPlan_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/GetPlan_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__GetPlan_Response() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__LoadMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__LoadMap_Request__init(msg: *mut LoadMap_Request) -> bool;
    fn nav_msgs__srv__LoadMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>, size: usize) -> bool;
    fn nav_msgs__srv__LoadMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>);
    fn nav_msgs__srv__LoadMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Request>) -> bool;
}

// Corresponds to nav_msgs__srv__LoadMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Request {
    pub map_url: rosidl_runtime_rs::String,
}



impl Default for LoadMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__LoadMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__LoadMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/LoadMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__LoadMap_Request() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__LoadMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__LoadMap_Response__init(msg: *mut LoadMap_Response) -> bool;
    fn nav_msgs__srv__LoadMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>, size: usize) -> bool;
    fn nav_msgs__srv__LoadMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>);
    fn nav_msgs__srv__LoadMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadMap_Response>) -> bool;
}

// Corresponds to nav_msgs__srv__LoadMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadMap_Response {
    pub map: crate::msg::rmw::OccupancyGrid,
    pub result: u8,
}

impl LoadMap_Response {
    pub const RESULT_SUCCESS: u8 = 0;
    pub const RESULT_MAP_DOES_NOT_EXIST: u8 = 1;
    pub const RESULT_INVALID_MAP_DATA: u8 = 2;
    pub const RESULT_INVALID_MAP_METADATA: u8 = 3;
    pub const RESULT_UNDEFINED_FAILURE: u8 = 255;
}


impl Default for LoadMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__LoadMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__LoadMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__LoadMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/LoadMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__LoadMap_Response() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__SetMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__SetMap_Request__init(msg: *mut SetMap_Request) -> bool;
    fn nav_msgs__srv__SetMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetMap_Request>, size: usize) -> bool;
    fn nav_msgs__srv__SetMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetMap_Request>);
    fn nav_msgs__srv__SetMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetMap_Request>) -> bool;
}

// Corresponds to nav_msgs__srv__SetMap_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetMap_Request {
    pub map: crate::msg::rmw::OccupancyGrid,
    pub initial_pose: geometry_msgs::msg::rmw::PoseWithCovarianceStamped,
}



impl Default for SetMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__SetMap_Request__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__SetMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/SetMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__SetMap_Request() }
  }
}


#[link(name = "nav_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__SetMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "nav_msgs__rosidl_generator_c")]
extern "C" {
    fn nav_msgs__srv__SetMap_Response__init(msg: *mut SetMap_Response) -> bool;
    fn nav_msgs__srv__SetMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetMap_Response>, size: usize) -> bool;
    fn nav_msgs__srv__SetMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetMap_Response>);
    fn nav_msgs__srv__SetMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetMap_Response>) -> bool;
}

// Corresponds to nav_msgs__srv__SetMap_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetMap_Response {
    pub success: bool,
}



impl Default for SetMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nav_msgs__srv__SetMap_Response__init(&mut msg as *mut _) {
        panic!("Call to nav_msgs__srv__SetMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nav_msgs__srv__SetMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "nav_msgs/srv/SetMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__srv__SetMap_Response() }
  }
}






  #[link(name = "nav_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetMap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav_msgs__srv__GetMap
  pub struct GetMap;

  impl rosidl_runtime_rs::Service for GetMap {
    type Request = crate::srv::rmw::GetMap_Request;
    type Response = crate::srv::rmw::GetMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetMap() }
    }
  }




  #[link(name = "nav_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetPlan() -> *const std::ffi::c_void;
  }

  // Corresponds to nav_msgs__srv__GetPlan
  pub struct GetPlan;

  impl rosidl_runtime_rs::Service for GetPlan {
    type Request = crate::srv::rmw::GetPlan_Request;
    type Response = crate::srv::rmw::GetPlan_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__GetPlan() }
    }
  }




  #[link(name = "nav_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__LoadMap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav_msgs__srv__LoadMap
  pub struct LoadMap;

  impl rosidl_runtime_rs::Service for LoadMap {
    type Request = crate::srv::rmw::LoadMap_Request;
    type Response = crate::srv::rmw::LoadMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__LoadMap() }
    }
  }




  #[link(name = "nav_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__SetMap() -> *const std::ffi::c_void;
  }

  // Corresponds to nav_msgs__srv__SetMap
  pub struct SetMap;

  impl rosidl_runtime_rs::Service for SetMap {
    type Request = crate::srv::rmw::SetMap_Request;
    type Response = crate::srv::rmw::SetMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__nav_msgs__srv__SetMap() }
    }
  }


}  // mod rmw
