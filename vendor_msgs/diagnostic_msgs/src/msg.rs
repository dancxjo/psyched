pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__msg__DiagnosticArray__init(msg: *mut DiagnosticArray) -> bool;
    fn diagnostic_msgs__msg__DiagnosticArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagnosticArray>, size: usize) -> bool;
    fn diagnostic_msgs__msg__DiagnosticArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagnosticArray>);
    fn diagnostic_msgs__msg__DiagnosticArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagnosticArray>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagnosticArray>) -> bool;
}

// Corresponds to diagnostic_msgs__msg__DiagnosticArray
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagnosticArray {
    pub header: std_msgs::msg::rmw::Header,
    pub status: rosidl_runtime_rs::Sequence<crate::msg::rmw::DiagnosticStatus>,
}



impl Default for DiagnosticArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__msg__DiagnosticArray__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__msg__DiagnosticArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagnosticArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagnosticArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagnosticArray where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/msg/DiagnosticArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticArray() }
  }
}


#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticStatus() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__msg__DiagnosticStatus__init(msg: *mut DiagnosticStatus) -> bool;
    fn diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DiagnosticStatus>, size: usize) -> bool;
    fn diagnostic_msgs__msg__DiagnosticStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DiagnosticStatus>);
    fn diagnostic_msgs__msg__DiagnosticStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DiagnosticStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<DiagnosticStatus>) -> bool;
}

// Corresponds to diagnostic_msgs__msg__DiagnosticStatus
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagnosticStatus {
    pub level: u8,
    pub name: rosidl_runtime_rs::String,
    pub message: rosidl_runtime_rs::String,
    pub hardware_id: rosidl_runtime_rs::String,
    pub values: rosidl_runtime_rs::Sequence<crate::msg::rmw::KeyValue>,
}

impl DiagnosticStatus {
    /// Possible levels of operations.
    pub const OK: u8 = 0;
    pub const WARN: u8 = 1;
    pub const ERROR: u8 = 2;
    pub const STALE: u8 = 3;
}


impl Default for DiagnosticStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__msg__DiagnosticStatus__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__msg__DiagnosticStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DiagnosticStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__DiagnosticStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DiagnosticStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DiagnosticStatus where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/msg/DiagnosticStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__DiagnosticStatus() }
  }
}


#[link(name = "diagnostic_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__KeyValue() -> *const std::ffi::c_void;
}

#[link(name = "diagnostic_msgs__rosidl_generator_c")]
extern "C" {
    fn diagnostic_msgs__msg__KeyValue__init(msg: *mut KeyValue) -> bool;
    fn diagnostic_msgs__msg__KeyValue__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<KeyValue>, size: usize) -> bool;
    fn diagnostic_msgs__msg__KeyValue__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<KeyValue>);
    fn diagnostic_msgs__msg__KeyValue__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<KeyValue>, out_seq: *mut rosidl_runtime_rs::Sequence<KeyValue>) -> bool;
}

// Corresponds to diagnostic_msgs__msg__KeyValue
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyValue {
    pub key: rosidl_runtime_rs::String,
    pub value: rosidl_runtime_rs::String,
}



impl Default for KeyValue {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !diagnostic_msgs__msg__KeyValue__init(&mut msg as *mut _) {
        panic!("Call to diagnostic_msgs__msg__KeyValue__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for KeyValue {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__KeyValue__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__KeyValue__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { diagnostic_msgs__msg__KeyValue__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for KeyValue {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for KeyValue where Self: Sized {
  const TYPE_NAME: &'static str = "diagnostic_msgs/msg/KeyValue";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__diagnostic_msgs__msg__KeyValue() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagnosticArray {
    pub header: std_msgs::msg::Header,
    pub status: Vec<crate::msg::DiagnosticStatus>,
}



impl Default for DiagnosticArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::DiagnosticArray::default())
  }
}

impl rosidl_runtime_rs::Message for DiagnosticArray {
  type RmwMsg = crate::msg::rmw::DiagnosticArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        status: msg.status
          .into_iter()
          .map(|elem| crate::msg::DiagnosticStatus::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        status: msg.status
          .iter()
          .map(|elem| crate::msg::DiagnosticStatus::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      status: msg.status
          .into_iter()
          .map(crate::msg::DiagnosticStatus::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DiagnosticStatus {
    pub level: u8,
    pub name: std::string::String,
    pub message: std::string::String,
    pub hardware_id: std::string::String,
    pub values: Vec<crate::msg::KeyValue>,
}

impl DiagnosticStatus {
    /// Possible levels of operations.
    pub const OK: u8 = 0;
    pub const WARN: u8 = 1;
    pub const ERROR: u8 = 2;
    pub const STALE: u8 = 3;
}


impl Default for DiagnosticStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::DiagnosticStatus::default())
  }
}

impl rosidl_runtime_rs::Message for DiagnosticStatus {
  type RmwMsg = crate::msg::rmw::DiagnosticStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        level: msg.level,
        name: msg.name.as_str().into(),
        message: msg.message.as_str().into(),
        hardware_id: msg.hardware_id.as_str().into(),
        values: msg.values
          .into_iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      level: msg.level,
        name: msg.name.as_str().into(),
        message: msg.message.as_str().into(),
        hardware_id: msg.hardware_id.as_str().into(),
        values: msg.values
          .iter()
          .map(|elem| crate::msg::KeyValue::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      level: msg.level,
      name: msg.name.to_string(),
      message: msg.message.to_string(),
      hardware_id: msg.hardware_id.to_string(),
      values: msg.values
          .into_iter()
          .map(crate::msg::KeyValue::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyValue {
    pub key: std::string::String,
    pub value: std::string::String,
}



impl Default for KeyValue {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::KeyValue::default())
  }
}

impl rosidl_runtime_rs::Message for KeyValue {
  type RmwMsg = crate::msg::rmw::KeyValue;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        key: msg.key.as_str().into(),
        value: msg.value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      key: msg.key.to_string(),
      value: msg.value.to_string(),
    }
  }
}


