

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfo_Request {
    pub camera_info: crate::msg::CameraInfo,
}



impl Default for SetCameraInfo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetCameraInfo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetCameraInfo_Request {
  type RmwMsg = crate::srv::rmw::SetCameraInfo_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        camera_info: crate::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Owned(msg.camera_info)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        camera_info: crate::msg::CameraInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.camera_info)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      camera_info: crate::msg::CameraInfo::from_rmw_message(msg.camera_info),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfo_Response {
    pub success: bool,
    pub status_message: std::string::String,
}



impl Default for SetCameraInfo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SetCameraInfo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetCameraInfo_Response {
  type RmwMsg = crate::srv::rmw::SetCameraInfo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        status_message: msg.status_message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        status_message: msg.status_message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      status_message: msg.status_message.to_string(),
    }
  }
}






#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__sensor_msgs__srv__SetCameraInfo() -> *const std::ffi::c_void;
}

// Corresponds to sensor_msgs__srv__SetCameraInfo
pub struct SetCameraInfo;

impl rosidl_runtime_rs::Service for SetCameraInfo {
  type Request = crate::srv::SetCameraInfo_Request;
  type Response = crate::srv::SetCameraInfo_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__sensor_msgs__srv__SetCameraInfo() }
  }
}




pub mod rmw {

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__srv__SetCameraInfo_Request() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__srv__SetCameraInfo_Request__init(msg: *mut SetCameraInfo_Request) -> bool;
    fn sensor_msgs__srv__SetCameraInfo_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Request>, size: usize) -> bool;
    fn sensor_msgs__srv__SetCameraInfo_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Request>);
    fn sensor_msgs__srv__SetCameraInfo_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCameraInfo_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Request>) -> bool;
}

// Corresponds to sensor_msgs__srv__SetCameraInfo_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfo_Request {
    pub camera_info: crate::msg::rmw::CameraInfo,
}



impl Default for SetCameraInfo_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__srv__SetCameraInfo_Request__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__srv__SetCameraInfo_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCameraInfo_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCameraInfo_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCameraInfo_Request where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/srv/SetCameraInfo_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__srv__SetCameraInfo_Request() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__srv__SetCameraInfo_Response() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__srv__SetCameraInfo_Response__init(msg: *mut SetCameraInfo_Response) -> bool;
    fn sensor_msgs__srv__SetCameraInfo_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Response>, size: usize) -> bool;
    fn sensor_msgs__srv__SetCameraInfo_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Response>);
    fn sensor_msgs__srv__SetCameraInfo_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetCameraInfo_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetCameraInfo_Response>) -> bool;
}

// Corresponds to sensor_msgs__srv__SetCameraInfo_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfo_Response {
    pub success: bool,
    pub status_message: rosidl_runtime_rs::String,
}



impl Default for SetCameraInfo_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__srv__SetCameraInfo_Response__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__srv__SetCameraInfo_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetCameraInfo_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__srv__SetCameraInfo_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetCameraInfo_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetCameraInfo_Response where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/srv/SetCameraInfo_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__srv__SetCameraInfo_Response() }
  }
}






  #[link(name = "sensor_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__sensor_msgs__srv__SetCameraInfo() -> *const std::ffi::c_void;
  }

  // Corresponds to sensor_msgs__srv__SetCameraInfo
  pub struct SetCameraInfo;

  impl rosidl_runtime_rs::Service for SetCameraInfo {
    type Request = crate::srv::rmw::SetCameraInfo_Request;
    type Response = crate::srv::rmw::SetCameraInfo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__sensor_msgs__srv__SetCameraInfo() }
    }
  }


}  // mod rmw
