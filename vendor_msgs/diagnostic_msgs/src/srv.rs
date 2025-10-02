

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddDiagnostics_Request {
    pub load_namespace: std::string::String,
}



impl Default for AddDiagnostics_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::AddDiagnostics_Request::default())
  }
}

impl rosidl_runtime_rs::Message for AddDiagnostics_Request {
  type RmwMsg = crate::srv::rmw::AddDiagnostics_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        load_namespace: msg.load_namespace.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        load_namespace: msg.load_namespace.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      load_namespace: msg.load_namespace.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddDiagnostics_Response {
    pub success: bool,
    pub message: std::string::String,
}



impl Default for AddDiagnostics_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::AddDiagnostics_Response::default())
  }
}

impl rosidl_runtime_rs::Message for AddDiagnostics_Response {
  type RmwMsg = crate::srv::rmw::AddDiagnostics_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelfTest_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SelfTest_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SelfTest_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SelfTest_Request {
  type RmwMsg = crate::srv::rmw::SelfTest_Request;

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
pub struct SelfTest_Response {
    pub id: std::string::String,
    pub passed: u8,
    pub status: Vec<crate::msg::DiagnosticStatus>,
}



impl Default for SelfTest_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::SelfTest_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SelfTest_Response {
  type RmwMsg = crate::srv::rmw::SelfTest_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id.as_str().into(),
        passed: msg.passed,
        status: msg.status
          .into_iter()
          .map(|elem| crate::msg::DiagnosticStatus::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id.as_str().into(),
      passed: msg.passed,
        status: msg.status
          .iter()
          .map(|elem| crate::msg::DiagnosticStatus::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id.to_string(),
      passed: msg.passed,
      status: msg.status
          .into_iter()
          .map(crate::msg::DiagnosticStatus::from_rmw_message)
          .collect(),
    }
  }
}






#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__AddDiagnostics() -> *const std::ffi::c_void;
}

// Corresponds to diagnostic_msgs__srv__AddDiagnostics
pub struct AddDiagnostics;

impl rosidl_runtime_rs::Service for AddDiagnostics {
  type Request = crate::srv::AddDiagnostics_Request;
  type Response = crate::srv::AddDiagnostics_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__AddDiagnostics() }
  }
}




#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__SelfTest() -> *const std::ffi::c_void;
}

// Corresponds to diagnostic_msgs__srv__SelfTest
pub struct SelfTest;

impl rosidl_runtime_rs::Service for SelfTest {
  type Request = crate::srv::SelfTest_Request;
  type Response = crate::srv::SelfTest_Response;

  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__SelfTest() }
  }
}




pub mod rmw {

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__AddDiagnostics_Request() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__srv__AddDiagnostics_Request__init(msg: *mut AddDiagnostics_Request) -> bool;
    fn diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Request>, size: usize) -> bool;
    fn diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Request>);
    fn diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AddDiagnostics_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Request>) -> bool;
}

// Corresponds to diagnostic_msgs__srv__AddDiagnostics_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddDiagnostics_Request {
    pub load_namespace: rosidl_runtime_rs::String,
}



impl Default for AddDiagnostics_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__srv__AddDiagnostics_Request__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__srv__AddDiagnostics_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AddDiagnostics_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AddDiagnostics_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AddDiagnostics_Request where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/srv/AddDiagnostics_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__AddDiagnostics_Request() }
  }
}


#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__AddDiagnostics_Response() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__srv__AddDiagnostics_Response__init(msg: *mut AddDiagnostics_Response) -> bool;
    fn diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Response>, size: usize) -> bool;
    fn diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Response>);
    fn diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AddDiagnostics_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<AddDiagnostics_Response>) -> bool;
}

// Corresponds to diagnostic_msgs__srv__AddDiagnostics_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AddDiagnostics_Response {
    pub success: bool,
    pub message: rosidl_runtime_rs::String,
}



impl Default for AddDiagnostics_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__srv__AddDiagnostics_Response__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__srv__AddDiagnostics_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AddDiagnostics_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__AddDiagnostics_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AddDiagnostics_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AddDiagnostics_Response where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/srv/AddDiagnostics_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__AddDiagnostics_Response() }
  }
}


#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__SelfTest_Request() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__srv__SelfTest_Request__init(msg: *mut SelfTest_Request) -> bool;
    fn diagnostic_msgs__srv__SelfTest_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Request>, size: usize) -> bool;
    fn diagnostic_msgs__srv__SelfTest_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Request>);
    fn diagnostic_msgs__srv__SelfTest_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SelfTest_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Request>) -> bool;
}

// Corresponds to diagnostic_msgs__srv__SelfTest_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelfTest_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for SelfTest_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__srv__SelfTest_Request__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__srv__SelfTest_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SelfTest_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SelfTest_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SelfTest_Request where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/srv/SelfTest_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__SelfTest_Request() }
  }
}


#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__SelfTest_Response() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__srv__SelfTest_Response__init(msg: *mut SelfTest_Response) -> bool;
    fn diagnostic_msgs__srv__SelfTest_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Response>, size: usize) -> bool;
    fn diagnostic_msgs__srv__SelfTest_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Response>);
    fn diagnostic_msgs__srv__SelfTest_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SelfTest_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SelfTest_Response>) -> bool;
}

// Corresponds to diagnostic_msgs__srv__SelfTest_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SelfTest_Response {
    pub id: rosidl_runtime_rs::String,
    pub passed: u8,
    pub status: rosidl_runtime_rs::Sequence<crate::msg::rmw::DiagnosticStatus>,
}



impl Default for SelfTest_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__srv__SelfTest_Response__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__srv__SelfTest_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SelfTest_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__srv__SelfTest_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SelfTest_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SelfTest_Response where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/srv/SelfTest_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__srv__SelfTest_Response() }
  }
}






  #[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__AddDiagnostics() -> *const std::ffi::c_void;
  }

  // Corresponds to diagnostic_msgs__srv__AddDiagnostics
  pub struct AddDiagnostics;

  impl rosidl_runtime_rs::Service for AddDiagnostics {
    type Request = crate::srv::rmw::AddDiagnostics_Request;
    type Response = crate::srv::rmw::AddDiagnostics_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__AddDiagnostics() }
    }
  }




  #[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__SelfTest() -> *const std::ffi::c_void;
  }

  // Corresponds to diagnostic_msgs__srv__SelfTest
  pub struct SelfTest;

  impl rosidl_runtime_rs::Service for SelfTest {
    type Request = crate::srv::rmw::SelfTest_Request;
    type Response = crate::srv::rmw::SelfTest_Response;

    fn get_type_support() -> *const std::ffi::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__diagnostic_msgs__srv__SelfTest() }
    }
  }


}  // mod rmw
