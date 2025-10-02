pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__BatteryState() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__BatteryState__init(msg: *mut BatteryState) -> bool;
    fn sensor_msgs__msg__BatteryState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BatteryState>, size: usize) -> bool;
    fn sensor_msgs__msg__BatteryState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BatteryState>);
    fn sensor_msgs__msg__BatteryState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BatteryState>, out_seq: *mut rosidl_runtime_rs::Sequence<BatteryState>) -> bool;
}

// Corresponds to sensor_msgs__msg__BatteryState
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BatteryState {
    pub header: std_msgs::msg::rmw::Header,
    pub voltage: f32,
    pub temperature: f32,
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    pub percentage: f32,
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    pub cell_voltage: rosidl_runtime_rs::Sequence<f32>,
    pub cell_temperature: rosidl_runtime_rs::Sequence<f32>,
    pub location: rosidl_runtime_rs::String,
    pub serial_number: rosidl_runtime_rs::String,
}

impl BatteryState {
    /// Constants are chosen to match the enums in the linux kernel
    /// defined in include/linux/power_supply.h as of version 3.7
    /// The one difference is for style reasons the constants are
    /// all uppercase not mixed case.
    /// Power supply status constants
    pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;
    /// Power supply health constants
    pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
    /// Power supply technology (chemistry) constants
    /// Unknown battery technology
    pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    /// Nickel-Metal Hydride battery
    pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    /// Lithium-ion battery
    pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    /// Lithium Polymer battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    /// Lithium Iron Phosphate battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    /// Nickel-Cadmium battery
    pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    /// Lithium Manganese Dioxide battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    /// Ternary Lithium battery
    pub const POWER_SUPPLY_TECHNOLOGY_TERNARY: u8 = 7;
    /// Valve Regulated Lead-Acid battery
    pub const POWER_SUPPLY_TECHNOLOGY_VRLA: u8 = 8;
}


impl Default for BatteryState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__BatteryState__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__BatteryState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BatteryState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__BatteryState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__BatteryState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__BatteryState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BatteryState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BatteryState where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/BatteryState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__BatteryState() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__CameraInfo() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__CameraInfo__init(msg: *mut CameraInfo) -> bool;
    fn sensor_msgs__msg__CameraInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CameraInfo>, size: usize) -> bool;
    fn sensor_msgs__msg__CameraInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CameraInfo>);
    fn sensor_msgs__msg__CameraInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CameraInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<CameraInfo>) -> bool;
}

// Corresponds to sensor_msgs__msg__CameraInfo
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CameraInfo {
    pub header: std_msgs::msg::rmw::Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: rosidl_runtime_rs::String,
    pub d: rosidl_runtime_rs::Sequence<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: crate::msg::rmw::RegionOfInterest,
}



impl Default for CameraInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__CameraInfo__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__CameraInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CameraInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CameraInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CameraInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CameraInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CameraInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CameraInfo where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/CameraInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__CameraInfo() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__ChannelFloat32() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__ChannelFloat32__init(msg: *mut ChannelFloat32) -> bool;
    fn sensor_msgs__msg__ChannelFloat32__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ChannelFloat32>, size: usize) -> bool;
    fn sensor_msgs__msg__ChannelFloat32__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ChannelFloat32>);
    fn sensor_msgs__msg__ChannelFloat32__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ChannelFloat32>, out_seq: *mut rosidl_runtime_rs::Sequence<ChannelFloat32>) -> bool;
}

// Corresponds to sensor_msgs__msg__ChannelFloat32
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChannelFloat32 {
    pub name: rosidl_runtime_rs::String,
    pub values: rosidl_runtime_rs::Sequence<f32>,
}



impl Default for ChannelFloat32 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__ChannelFloat32__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__ChannelFloat32__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ChannelFloat32 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__ChannelFloat32__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__ChannelFloat32__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__ChannelFloat32__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ChannelFloat32 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ChannelFloat32 where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/ChannelFloat32";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__ChannelFloat32() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__CompressedImage() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__CompressedImage__init(msg: *mut CompressedImage) -> bool;
    fn sensor_msgs__msg__CompressedImage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CompressedImage>, size: usize) -> bool;
    fn sensor_msgs__msg__CompressedImage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CompressedImage>);
    fn sensor_msgs__msg__CompressedImage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CompressedImage>, out_seq: *mut rosidl_runtime_rs::Sequence<CompressedImage>) -> bool;
}

// Corresponds to sensor_msgs__msg__CompressedImage
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CompressedImage {
    pub header: std_msgs::msg::rmw::Header,
    pub format: rosidl_runtime_rs::String,
    pub data: rosidl_runtime_rs::Sequence<u8>,
}



impl Default for CompressedImage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__CompressedImage__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__CompressedImage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CompressedImage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CompressedImage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CompressedImage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__CompressedImage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CompressedImage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CompressedImage where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/CompressedImage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__CompressedImage() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__FluidPressure() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__FluidPressure__init(msg: *mut FluidPressure) -> bool;
    fn sensor_msgs__msg__FluidPressure__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FluidPressure>, size: usize) -> bool;
    fn sensor_msgs__msg__FluidPressure__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FluidPressure>);
    fn sensor_msgs__msg__FluidPressure__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FluidPressure>, out_seq: *mut rosidl_runtime_rs::Sequence<FluidPressure>) -> bool;
}

// Corresponds to sensor_msgs__msg__FluidPressure
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FluidPressure {
    pub header: std_msgs::msg::rmw::Header,
    pub fluid_pressure: f64,
    pub variance: f64,
}



impl Default for FluidPressure {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__FluidPressure__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__FluidPressure__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FluidPressure {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__FluidPressure__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__FluidPressure__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__FluidPressure__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FluidPressure {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FluidPressure where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/FluidPressure";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__FluidPressure() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Illuminance() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Illuminance__init(msg: *mut Illuminance) -> bool;
    fn sensor_msgs__msg__Illuminance__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Illuminance>, size: usize) -> bool;
    fn sensor_msgs__msg__Illuminance__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Illuminance>);
    fn sensor_msgs__msg__Illuminance__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Illuminance>, out_seq: *mut rosidl_runtime_rs::Sequence<Illuminance>) -> bool;
}

// Corresponds to sensor_msgs__msg__Illuminance
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Illuminance {
    pub header: std_msgs::msg::rmw::Header,
    pub illuminance: f64,
    pub variance: f64,
}



impl Default for Illuminance {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Illuminance__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Illuminance__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Illuminance {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Illuminance__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Illuminance__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Illuminance__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Illuminance {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Illuminance where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Illuminance";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Illuminance() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Image__init(msg: *mut Image) -> bool;
    fn sensor_msgs__msg__Image__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Image>, size: usize) -> bool;
    fn sensor_msgs__msg__Image__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Image>);
    fn sensor_msgs__msg__Image__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Image>, out_seq: *mut rosidl_runtime_rs::Sequence<Image>) -> bool;
}

// Corresponds to sensor_msgs__msg__Image
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Image {
    pub header: std_msgs::msg::rmw::Header,
    pub height: u32,
    pub width: u32,
    pub encoding: rosidl_runtime_rs::String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: rosidl_runtime_rs::Sequence<u8>,
}



impl Default for Image {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Image__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Image__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Image {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Image__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Image__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Image__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Image {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Image where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Image";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Image() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Imu() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Imu__init(msg: *mut Imu) -> bool;
    fn sensor_msgs__msg__Imu__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Imu>, size: usize) -> bool;
    fn sensor_msgs__msg__Imu__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Imu>);
    fn sensor_msgs__msg__Imu__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Imu>, out_seq: *mut rosidl_runtime_rs::Sequence<Imu>) -> bool;
}

// Corresponds to sensor_msgs__msg__Imu
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Imu {
    pub header: std_msgs::msg::rmw::Header,
    pub orientation: geometry_msgs::msg::rmw::Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: geometry_msgs::msg::rmw::Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: geometry_msgs::msg::rmw::Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}



impl Default for Imu {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Imu__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Imu__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Imu {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Imu__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Imu__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Imu__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Imu {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Imu where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Imu";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Imu() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JointState() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__JointState__init(msg: *mut JointState) -> bool;
    fn sensor_msgs__msg__JointState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointState>, size: usize) -> bool;
    fn sensor_msgs__msg__JointState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointState>);
    fn sensor_msgs__msg__JointState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointState>, out_seq: *mut rosidl_runtime_rs::Sequence<JointState>) -> bool;
}

// Corresponds to sensor_msgs__msg__JointState
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointState {
    pub header: std_msgs::msg::rmw::Header,
    pub name: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub position: rosidl_runtime_rs::Sequence<f64>,
    pub velocity: rosidl_runtime_rs::Sequence<f64>,
    pub effort: rosidl_runtime_rs::Sequence<f64>,
}



impl Default for JointState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__JointState__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__JointState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JointState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JointState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JointState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointState where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/JointState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JointState() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Joy() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Joy__init(msg: *mut Joy) -> bool;
    fn sensor_msgs__msg__Joy__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Joy>, size: usize) -> bool;
    fn sensor_msgs__msg__Joy__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Joy>);
    fn sensor_msgs__msg__Joy__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Joy>, out_seq: *mut rosidl_runtime_rs::Sequence<Joy>) -> bool;
}

// Corresponds to sensor_msgs__msg__Joy
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Joy {
    pub header: std_msgs::msg::rmw::Header,
    pub axes: rosidl_runtime_rs::Sequence<f32>,
    pub buttons: rosidl_runtime_rs::Sequence<i32>,
}



impl Default for Joy {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Joy__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Joy__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Joy {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Joy__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Joy__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Joy__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Joy {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Joy where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Joy";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Joy() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JoyFeedback() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__JoyFeedback__init(msg: *mut JoyFeedback) -> bool;
    fn sensor_msgs__msg__JoyFeedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JoyFeedback>, size: usize) -> bool;
    fn sensor_msgs__msg__JoyFeedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JoyFeedback>);
    fn sensor_msgs__msg__JoyFeedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JoyFeedback>, out_seq: *mut rosidl_runtime_rs::Sequence<JoyFeedback>) -> bool;
}

// Corresponds to sensor_msgs__msg__JoyFeedback
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedback {
    pub type_: u8,
    pub id: u8,
    pub intensity: f32,
}

impl JoyFeedback {
    pub const TYPE_LED: u8 = 0;
    pub const TYPE_RUMBLE: u8 = 1;
    pub const TYPE_BUZZER: u8 = 2;
}


impl Default for JoyFeedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__JoyFeedback__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__JoyFeedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JoyFeedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JoyFeedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JoyFeedback where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/JoyFeedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JoyFeedback() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JoyFeedbackArray() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__JoyFeedbackArray__init(msg: *mut JoyFeedbackArray) -> bool;
    fn sensor_msgs__msg__JoyFeedbackArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JoyFeedbackArray>, size: usize) -> bool;
    fn sensor_msgs__msg__JoyFeedbackArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JoyFeedbackArray>);
    fn sensor_msgs__msg__JoyFeedbackArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JoyFeedbackArray>, out_seq: *mut rosidl_runtime_rs::Sequence<JoyFeedbackArray>) -> bool;
}

// Corresponds to sensor_msgs__msg__JoyFeedbackArray
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedbackArray {
    pub array: rosidl_runtime_rs::Sequence<crate::msg::rmw::JoyFeedback>,
}



impl Default for JoyFeedbackArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__JoyFeedbackArray__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__JoyFeedbackArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JoyFeedbackArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedbackArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedbackArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__JoyFeedbackArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JoyFeedbackArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JoyFeedbackArray where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/JoyFeedbackArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JoyFeedbackArray() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__LaserEcho() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__LaserEcho__init(msg: *mut LaserEcho) -> bool;
    fn sensor_msgs__msg__LaserEcho__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaserEcho>, size: usize) -> bool;
    fn sensor_msgs__msg__LaserEcho__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaserEcho>);
    fn sensor_msgs__msg__LaserEcho__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaserEcho>, out_seq: *mut rosidl_runtime_rs::Sequence<LaserEcho>) -> bool;
}

// Corresponds to sensor_msgs__msg__LaserEcho
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserEcho {
    pub echoes: rosidl_runtime_rs::Sequence<f32>,
}



impl Default for LaserEcho {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__LaserEcho__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__LaserEcho__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaserEcho {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserEcho__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserEcho__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserEcho__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaserEcho {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaserEcho where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/LaserEcho";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__LaserEcho() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__LaserScan() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__LaserScan__init(msg: *mut LaserScan) -> bool;
    fn sensor_msgs__msg__LaserScan__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaserScan>, size: usize) -> bool;
    fn sensor_msgs__msg__LaserScan__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaserScan>);
    fn sensor_msgs__msg__LaserScan__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaserScan>, out_seq: *mut rosidl_runtime_rs::Sequence<LaserScan>) -> bool;
}

// Corresponds to sensor_msgs__msg__LaserScan
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserScan {
    pub header: std_msgs::msg::rmw::Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: rosidl_runtime_rs::Sequence<f32>,
    pub intensities: rosidl_runtime_rs::Sequence<f32>,
}



impl Default for LaserScan {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__LaserScan__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__LaserScan__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaserScan {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserScan__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserScan__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__LaserScan__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaserScan {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaserScan where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/LaserScan";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__LaserScan() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MagneticField() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__MagneticField__init(msg: *mut MagneticField) -> bool;
    fn sensor_msgs__msg__MagneticField__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MagneticField>, size: usize) -> bool;
    fn sensor_msgs__msg__MagneticField__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MagneticField>);
    fn sensor_msgs__msg__MagneticField__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MagneticField>, out_seq: *mut rosidl_runtime_rs::Sequence<MagneticField>) -> bool;
}

// Corresponds to sensor_msgs__msg__MagneticField
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MagneticField {
    pub header: std_msgs::msg::rmw::Header,
    pub magnetic_field: geometry_msgs::msg::rmw::Vector3,
    pub magnetic_field_covariance: [f64; 9],
}



impl Default for MagneticField {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__MagneticField__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__MagneticField__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MagneticField {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MagneticField__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MagneticField__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MagneticField__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MagneticField {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MagneticField where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/MagneticField";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MagneticField() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MultiDOFJointState() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__MultiDOFJointState__init(msg: *mut MultiDOFJointState) -> bool;
    fn sensor_msgs__msg__MultiDOFJointState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointState>, size: usize) -> bool;
    fn sensor_msgs__msg__MultiDOFJointState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointState>);
    fn sensor_msgs__msg__MultiDOFJointState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiDOFJointState>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiDOFJointState>) -> bool;
}

// Corresponds to sensor_msgs__msg__MultiDOFJointState
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointState {
    pub header: std_msgs::msg::rmw::Header,
    pub joint_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,
    pub transforms: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Transform>,
    pub twist: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Twist>,
    pub wrench: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Wrench>,
}



impl Default for MultiDOFJointState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__MultiDOFJointState__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__MultiDOFJointState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiDOFJointState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiDOFJointState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiDOFJointState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiDOFJointState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiDOFJointState where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/MultiDOFJointState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MultiDOFJointState() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MultiEchoLaserScan() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__MultiEchoLaserScan__init(msg: *mut MultiEchoLaserScan) -> bool;
    fn sensor_msgs__msg__MultiEchoLaserScan__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MultiEchoLaserScan>, size: usize) -> bool;
    fn sensor_msgs__msg__MultiEchoLaserScan__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MultiEchoLaserScan>);
    fn sensor_msgs__msg__MultiEchoLaserScan__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MultiEchoLaserScan>, out_seq: *mut rosidl_runtime_rs::Sequence<MultiEchoLaserScan>) -> bool;
}

// Corresponds to sensor_msgs__msg__MultiEchoLaserScan
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiEchoLaserScan {
    pub header: std_msgs::msg::rmw::Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: rosidl_runtime_rs::Sequence<crate::msg::rmw::LaserEcho>,
    pub intensities: rosidl_runtime_rs::Sequence<crate::msg::rmw::LaserEcho>,
}



impl Default for MultiEchoLaserScan {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__MultiEchoLaserScan__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__MultiEchoLaserScan__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MultiEchoLaserScan {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiEchoLaserScan__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiEchoLaserScan__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__MultiEchoLaserScan__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MultiEchoLaserScan {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MultiEchoLaserScan where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/MultiEchoLaserScan";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__MultiEchoLaserScan() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__NavSatFix() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__NavSatFix__init(msg: *mut NavSatFix) -> bool;
    fn sensor_msgs__msg__NavSatFix__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSatFix>, size: usize) -> bool;
    fn sensor_msgs__msg__NavSatFix__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSatFix>);
    fn sensor_msgs__msg__NavSatFix__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSatFix>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSatFix>) -> bool;
}

// Corresponds to sensor_msgs__msg__NavSatFix
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatFix {
    pub header: std_msgs::msg::rmw::Header,
    pub status: crate::msg::rmw::NavSatStatus,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}

impl NavSatFix {
    /// If the covariance of the fix is known, fill it in completely. If the
    /// GPS receiver provides the variance of each measurement, put them
    /// along the diagonal. If only Dilution of Precision is available,
    /// estimate an approximate covariance from that.
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}


impl Default for NavSatFix {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__NavSatFix__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__NavSatFix__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSatFix {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatFix__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatFix__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatFix__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSatFix {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSatFix where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/NavSatFix";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__NavSatFix() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__NavSatStatus() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__NavSatStatus__init(msg: *mut NavSatStatus) -> bool;
    fn sensor_msgs__msg__NavSatStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSatStatus>, size: usize) -> bool;
    fn sensor_msgs__msg__NavSatStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSatStatus>);
    fn sensor_msgs__msg__NavSatStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSatStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSatStatus>) -> bool;
}

// Corresponds to sensor_msgs__msg__NavSatStatus
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatStatus {
    pub status: i8,
    pub service: u16,
}

impl NavSatStatus {
    /// status is not yet set
    pub const STATUS_UNKNOWN: i8 = -2;
    /// unable to fix position
    pub const STATUS_NO_FIX: i8 = -1;
    /// unaugmented fix
    pub const STATUS_FIX: i8 = 0;
    /// with satellite-based augmentation
    pub const STATUS_SBAS_FIX: i8 = 1;
    /// with ground-based augmentation
    pub const STATUS_GBAS_FIX: i8 = 2;
    /// Bits defining which Global Navigation Satellite System signals were
    /// used by the receiver.
    /// Remember service is a bitfield, so checking (service & SERVICE_UNKNOWN) will not work. Use == instead.
    pub const SERVICE_UNKNOWN: u16 = 0;
    pub const SERVICE_GPS: u16 = 1;
    pub const SERVICE_GLONASS: u16 = 2;
    /// includes BeiDou.
    pub const SERVICE_COMPASS: u16 = 4;
    pub const SERVICE_GALILEO: u16 = 8;
}


impl Default for NavSatStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__NavSatStatus__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__NavSatStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSatStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__NavSatStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSatStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSatStatus where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/NavSatStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__NavSatStatus() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__PointCloud__init(msg: *mut PointCloud) -> bool;
    fn sensor_msgs__msg__PointCloud__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PointCloud>, size: usize) -> bool;
    fn sensor_msgs__msg__PointCloud__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PointCloud>);
    fn sensor_msgs__msg__PointCloud__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PointCloud>, out_seq: *mut rosidl_runtime_rs::Sequence<PointCloud>) -> bool;
}

// Corresponds to sensor_msgs__msg__PointCloud
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointCloud {
    pub header: std_msgs::msg::rmw::Header,
    pub points: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point32>,
    pub channels: rosidl_runtime_rs::Sequence<crate::msg::rmw::ChannelFloat32>,
}



impl Default for PointCloud {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__PointCloud__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__PointCloud__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PointCloud {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PointCloud {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PointCloud where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/PointCloud";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud2() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__PointCloud2__init(msg: *mut PointCloud2) -> bool;
    fn sensor_msgs__msg__PointCloud2__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PointCloud2>, size: usize) -> bool;
    fn sensor_msgs__msg__PointCloud2__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PointCloud2>);
    fn sensor_msgs__msg__PointCloud2__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PointCloud2>, out_seq: *mut rosidl_runtime_rs::Sequence<PointCloud2>) -> bool;
}

// Corresponds to sensor_msgs__msg__PointCloud2
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointCloud2 {
    pub header: std_msgs::msg::rmw::Header,
    pub height: u32,
    pub width: u32,
    pub fields: rosidl_runtime_rs::Sequence<crate::msg::rmw::PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: rosidl_runtime_rs::Sequence<u8>,
    pub is_dense: bool,
}



impl Default for PointCloud2 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__PointCloud2__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__PointCloud2__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PointCloud2 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud2__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud2__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointCloud2__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PointCloud2 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PointCloud2 where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/PointCloud2";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointCloud2() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointField() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__PointField__init(msg: *mut PointField) -> bool;
    fn sensor_msgs__msg__PointField__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PointField>, size: usize) -> bool;
    fn sensor_msgs__msg__PointField__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PointField>);
    fn sensor_msgs__msg__PointField__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PointField>, out_seq: *mut rosidl_runtime_rs::Sequence<PointField>) -> bool;
}

// Corresponds to sensor_msgs__msg__PointField
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointField {
    pub name: rosidl_runtime_rs::String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

impl PointField {
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const INT32: u8 = 5;
    pub const UINT32: u8 = 6;
    pub const FLOAT32: u8 = 7;
    pub const FLOAT64: u8 = 8;
    pub const INT64: u8 = 9;
    pub const UINT64: u8 = 10;
    pub const BOOL: u8 = 11;
}


impl Default for PointField {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__PointField__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__PointField__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PointField {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointField__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointField__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__PointField__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PointField {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PointField where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/PointField";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__PointField() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Range() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Range__init(msg: *mut Range) -> bool;
    fn sensor_msgs__msg__Range__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Range>, size: usize) -> bool;
    fn sensor_msgs__msg__Range__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Range>);
    fn sensor_msgs__msg__Range__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Range>, out_seq: *mut rosidl_runtime_rs::Sequence<Range>) -> bool;
}

// Corresponds to sensor_msgs__msg__Range
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Range {
    pub header: std_msgs::msg::rmw::Header,
    pub radiation_type: u8,
    pub field_of_view: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range: f32,
    pub variance: f32,
}

impl Range {
    /// Radiation type enums
    /// If you want a value added to this list, send an email to the ros-users list
    pub const ULTRASOUND: u8 = 0;
    pub const INFRARED: u8 = 1;
}


impl Default for Range {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Range__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Range__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Range {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Range__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Range__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Range__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Range {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Range where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Range";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Range() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__RegionOfInterest() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__RegionOfInterest__init(msg: *mut RegionOfInterest) -> bool;
    fn sensor_msgs__msg__RegionOfInterest__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RegionOfInterest>, size: usize) -> bool;
    fn sensor_msgs__msg__RegionOfInterest__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RegionOfInterest>);
    fn sensor_msgs__msg__RegionOfInterest__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RegionOfInterest>, out_seq: *mut rosidl_runtime_rs::Sequence<RegionOfInterest>) -> bool;
}

// Corresponds to sensor_msgs__msg__RegionOfInterest
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}



impl Default for RegionOfInterest {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__RegionOfInterest__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__RegionOfInterest__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RegionOfInterest {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RegionOfInterest__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RegionOfInterest__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RegionOfInterest__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RegionOfInterest {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RegionOfInterest where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/RegionOfInterest";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__RegionOfInterest() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__RelativeHumidity() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__RelativeHumidity__init(msg: *mut RelativeHumidity) -> bool;
    fn sensor_msgs__msg__RelativeHumidity__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RelativeHumidity>, size: usize) -> bool;
    fn sensor_msgs__msg__RelativeHumidity__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RelativeHumidity>);
    fn sensor_msgs__msg__RelativeHumidity__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RelativeHumidity>, out_seq: *mut rosidl_runtime_rs::Sequence<RelativeHumidity>) -> bool;
}

// Corresponds to sensor_msgs__msg__RelativeHumidity
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RelativeHumidity {
    pub header: std_msgs::msg::rmw::Header,
    pub relative_humidity: f64,
    pub variance: f64,
}



impl Default for RelativeHumidity {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__RelativeHumidity__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__RelativeHumidity__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RelativeHumidity {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RelativeHumidity__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RelativeHumidity__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__RelativeHumidity__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RelativeHumidity {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RelativeHumidity where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/RelativeHumidity";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__RelativeHumidity() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Temperature() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__Temperature__init(msg: *mut Temperature) -> bool;
    fn sensor_msgs__msg__Temperature__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Temperature>, size: usize) -> bool;
    fn sensor_msgs__msg__Temperature__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Temperature>);
    fn sensor_msgs__msg__Temperature__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Temperature>, out_seq: *mut rosidl_runtime_rs::Sequence<Temperature>) -> bool;
}

// Corresponds to sensor_msgs__msg__Temperature
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Temperature {
    pub header: std_msgs::msg::rmw::Header,
    pub temperature: f64,
    pub variance: f64,
}



impl Default for Temperature {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__Temperature__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__Temperature__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Temperature {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Temperature__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Temperature__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__Temperature__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Temperature {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Temperature where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/Temperature";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Temperature() }
  }
}


#[link(name = "sensor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__TimeReference() -> *const std::ffi::c_void;
}

#[link(name = "sensor_msgs__rosidl_generator_c")]
extern "C" {
    fn sensor_msgs__msg__TimeReference__init(msg: *mut TimeReference) -> bool;
    fn sensor_msgs__msg__TimeReference__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TimeReference>, size: usize) -> bool;
    fn sensor_msgs__msg__TimeReference__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TimeReference>);
    fn sensor_msgs__msg__TimeReference__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TimeReference>, out_seq: *mut rosidl_runtime_rs::Sequence<TimeReference>) -> bool;
}

// Corresponds to sensor_msgs__msg__TimeReference
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TimeReference {
    pub header: std_msgs::msg::rmw::Header,
    pub time_ref: builtin_interfaces::msg::rmw::Time,
    pub source: rosidl_runtime_rs::String,
}



impl Default for TimeReference {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !sensor_msgs__msg__TimeReference__init(&mut msg as *mut _) {
        panic!("Call to sensor_msgs__msg__TimeReference__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TimeReference {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__TimeReference__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__TimeReference__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { sensor_msgs__msg__TimeReference__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TimeReference {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TimeReference where Self: Sized {
  const TYPE_NAME: &'static str = "sensor_msgs/msg/TimeReference";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__TimeReference() }
  }
}


}  // mod rmw


#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BatteryState {
    pub header: std_msgs::msg::Header,
    pub voltage: f32,
    pub temperature: f32,
    pub current: f32,
    pub charge: f32,
    pub capacity: f32,
    pub design_capacity: f32,
    pub percentage: f32,
    pub power_supply_status: u8,
    pub power_supply_health: u8,
    pub power_supply_technology: u8,
    pub present: bool,
    pub cell_voltage: Vec<f32>,
    pub cell_temperature: Vec<f32>,
    pub location: std::string::String,
    pub serial_number: std::string::String,
}

impl BatteryState {
    /// Constants are chosen to match the enums in the linux kernel
    /// defined in include/linux/power_supply.h as of version 3.7
    /// The one difference is for style reasons the constants are
    /// all uppercase not mixed case.
    /// Power supply status constants
    pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;
    /// Power supply health constants
    pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
    /// Power supply technology (chemistry) constants
    /// Unknown battery technology
    pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    /// Nickel-Metal Hydride battery
    pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    /// Lithium-ion battery
    pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    /// Lithium Polymer battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    /// Lithium Iron Phosphate battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    /// Nickel-Cadmium battery
    pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    /// Lithium Manganese Dioxide battery
    pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    /// Ternary Lithium battery
    pub const POWER_SUPPLY_TECHNOLOGY_TERNARY: u8 = 7;
    /// Valve Regulated Lead-Acid battery
    pub const POWER_SUPPLY_TECHNOLOGY_VRLA: u8 = 8;
}


impl Default for BatteryState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::BatteryState::default())
  }
}

impl rosidl_runtime_rs::Message for BatteryState {
  type RmwMsg = crate::msg::rmw::BatteryState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        voltage: msg.voltage,
        temperature: msg.temperature,
        current: msg.current,
        charge: msg.charge,
        capacity: msg.capacity,
        design_capacity: msg.design_capacity,
        percentage: msg.percentage,
        power_supply_status: msg.power_supply_status,
        power_supply_health: msg.power_supply_health,
        power_supply_technology: msg.power_supply_technology,
        present: msg.present,
        cell_voltage: msg.cell_voltage.into(),
        cell_temperature: msg.cell_temperature.into(),
        location: msg.location.as_str().into(),
        serial_number: msg.serial_number.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      voltage: msg.voltage,
      temperature: msg.temperature,
      current: msg.current,
      charge: msg.charge,
      capacity: msg.capacity,
      design_capacity: msg.design_capacity,
      percentage: msg.percentage,
      power_supply_status: msg.power_supply_status,
      power_supply_health: msg.power_supply_health,
      power_supply_technology: msg.power_supply_technology,
      present: msg.present,
        cell_voltage: msg.cell_voltage.as_slice().into(),
        cell_temperature: msg.cell_temperature.as_slice().into(),
        location: msg.location.as_str().into(),
        serial_number: msg.serial_number.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      voltage: msg.voltage,
      temperature: msg.temperature,
      current: msg.current,
      charge: msg.charge,
      capacity: msg.capacity,
      design_capacity: msg.design_capacity,
      percentage: msg.percentage,
      power_supply_status: msg.power_supply_status,
      power_supply_health: msg.power_supply_health,
      power_supply_technology: msg.power_supply_technology,
      present: msg.present,
      cell_voltage: msg.cell_voltage
          .into_iter()
          .collect(),
      cell_temperature: msg.cell_temperature
          .into_iter()
          .collect(),
      location: msg.location.to_string(),
      serial_number: msg.serial_number.to_string(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CameraInfo {
    pub header: std_msgs::msg::Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: std::string::String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: crate::msg::RegionOfInterest,
}



impl Default for CameraInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CameraInfo::default())
  }
}

impl rosidl_runtime_rs::Message for CameraInfo {
  type RmwMsg = crate::msg::rmw::CameraInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        height: msg.height,
        width: msg.width,
        distortion_model: msg.distortion_model.as_str().into(),
        d: msg.d.into(),
        k: msg.k,
        r: msg.r,
        p: msg.p,
        binning_x: msg.binning_x,
        binning_y: msg.binning_y,
        roi: crate::msg::RegionOfInterest::into_rmw_message(std::borrow::Cow::Owned(msg.roi)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      height: msg.height,
      width: msg.width,
        distortion_model: msg.distortion_model.as_str().into(),
        d: msg.d.as_slice().into(),
        k: msg.k,
        r: msg.r,
        p: msg.p,
      binning_x: msg.binning_x,
      binning_y: msg.binning_y,
        roi: crate::msg::RegionOfInterest::into_rmw_message(std::borrow::Cow::Borrowed(&msg.roi)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      height: msg.height,
      width: msg.width,
      distortion_model: msg.distortion_model.to_string(),
      d: msg.d
          .into_iter()
          .collect(),
      k: msg.k,
      r: msg.r,
      p: msg.p,
      binning_x: msg.binning_x,
      binning_y: msg.binning_y,
      roi: crate::msg::RegionOfInterest::from_rmw_message(msg.roi),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChannelFloat32 {
    pub name: std::string::String,
    pub values: Vec<f32>,
}



impl Default for ChannelFloat32 {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::ChannelFloat32::default())
  }
}

impl rosidl_runtime_rs::Message for ChannelFloat32 {
  type RmwMsg = crate::msg::rmw::ChannelFloat32;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        values: msg.values.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        values: msg.values.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      values: msg.values
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CompressedImage {
    pub header: std_msgs::msg::Header,
    pub format: std::string::String,
    pub data: Vec<u8>,
}



impl Default for CompressedImage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::CompressedImage::default())
  }
}

impl rosidl_runtime_rs::Message for CompressedImage {
  type RmwMsg = crate::msg::rmw::CompressedImage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        format: msg.format.as_str().into(),
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        format: msg.format.as_str().into(),
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      format: msg.format.to_string(),
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FluidPressure {
    pub header: std_msgs::msg::Header,
    pub fluid_pressure: f64,
    pub variance: f64,
}



impl Default for FluidPressure {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::FluidPressure::default())
  }
}

impl rosidl_runtime_rs::Message for FluidPressure {
  type RmwMsg = crate::msg::rmw::FluidPressure;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fluid_pressure: msg.fluid_pressure,
        variance: msg.variance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      fluid_pressure: msg.fluid_pressure,
      variance: msg.variance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fluid_pressure: msg.fluid_pressure,
      variance: msg.variance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Illuminance {
    pub header: std_msgs::msg::Header,
    pub illuminance: f64,
    pub variance: f64,
}



impl Default for Illuminance {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Illuminance::default())
  }
}

impl rosidl_runtime_rs::Message for Illuminance {
  type RmwMsg = crate::msg::rmw::Illuminance;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        illuminance: msg.illuminance,
        variance: msg.variance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      illuminance: msg.illuminance,
      variance: msg.variance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      illuminance: msg.illuminance,
      variance: msg.variance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Image {
    pub header: std_msgs::msg::Header,
    pub height: u32,
    pub width: u32,
    pub encoding: std::string::String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}



impl Default for Image {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Image::default())
  }
}

impl rosidl_runtime_rs::Message for Image {
  type RmwMsg = crate::msg::rmw::Image;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        height: msg.height,
        width: msg.width,
        encoding: msg.encoding.as_str().into(),
        is_bigendian: msg.is_bigendian,
        step: msg.step,
        data: msg.data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      height: msg.height,
      width: msg.width,
        encoding: msg.encoding.as_str().into(),
      is_bigendian: msg.is_bigendian,
      step: msg.step,
        data: msg.data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      height: msg.height,
      width: msg.width,
      encoding: msg.encoding.to_string(),
      is_bigendian: msg.is_bigendian,
      step: msg.step,
      data: msg.data
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Imu {
    pub header: std_msgs::msg::Header,
    pub orientation: geometry_msgs::msg::Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: geometry_msgs::msg::Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: geometry_msgs::msg::Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}



impl Default for Imu {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Imu::default())
  }
}

impl rosidl_runtime_rs::Message for Imu {
  type RmwMsg = crate::msg::rmw::Imu;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.orientation)).into_owned(),
        orientation_covariance: msg.orientation_covariance,
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.angular_velocity)).into_owned(),
        angular_velocity_covariance: msg.angular_velocity_covariance,
        linear_acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.linear_acceleration)).into_owned(),
        linear_acceleration_covariance: msg.linear_acceleration_covariance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        orientation: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.orientation)).into_owned(),
        orientation_covariance: msg.orientation_covariance,
        angular_velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.angular_velocity)).into_owned(),
        angular_velocity_covariance: msg.angular_velocity_covariance,
        linear_acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.linear_acceleration)).into_owned(),
        linear_acceleration_covariance: msg.linear_acceleration_covariance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      orientation: geometry_msgs::msg::Quaternion::from_rmw_message(msg.orientation),
      orientation_covariance: msg.orientation_covariance,
      angular_velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.angular_velocity),
      angular_velocity_covariance: msg.angular_velocity_covariance,
      linear_acceleration: geometry_msgs::msg::Vector3::from_rmw_message(msg.linear_acceleration),
      linear_acceleration_covariance: msg.linear_acceleration_covariance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointState {
    pub header: std_msgs::msg::Header,
    pub name: Vec<std::string::String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}



impl Default for JointState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JointState::default())
  }
}

impl rosidl_runtime_rs::Message for JointState {
  type RmwMsg = crate::msg::rmw::JointState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        name: msg.name
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        position: msg.position.into(),
        velocity: msg.velocity.into(),
        effort: msg.effort.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        name: msg.name
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        position: msg.position.as_slice().into(),
        velocity: msg.velocity.as_slice().into(),
        effort: msg.effort.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      name: msg.name
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      position: msg.position
          .into_iter()
          .collect(),
      velocity: msg.velocity
          .into_iter()
          .collect(),
      effort: msg.effort
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Joy {
    pub header: std_msgs::msg::Header,
    pub axes: Vec<f32>,
    pub buttons: Vec<i32>,
}



impl Default for Joy {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Joy::default())
  }
}

impl rosidl_runtime_rs::Message for Joy {
  type RmwMsg = crate::msg::rmw::Joy;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        axes: msg.axes.into(),
        buttons: msg.buttons.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        axes: msg.axes.as_slice().into(),
        buttons: msg.buttons.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      axes: msg.axes
          .into_iter()
          .collect(),
      buttons: msg.buttons
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedback {
    pub type_: u8,
    pub id: u8,
    pub intensity: f32,
}

impl JoyFeedback {
    pub const TYPE_LED: u8 = 0;
    pub const TYPE_RUMBLE: u8 = 1;
    pub const TYPE_BUZZER: u8 = 2;
}


impl Default for JoyFeedback {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JoyFeedback::default())
  }
}

impl rosidl_runtime_rs::Message for JoyFeedback {
  type RmwMsg = crate::msg::rmw::JoyFeedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        type_: msg.type_,
        id: msg.id,
        intensity: msg.intensity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      type_: msg.type_,
      id: msg.id,
      intensity: msg.intensity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      type_: msg.type_,
      id: msg.id,
      intensity: msg.intensity,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedbackArray {
    pub array: Vec<crate::msg::JoyFeedback>,
}



impl Default for JoyFeedbackArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::JoyFeedbackArray::default())
  }
}

impl rosidl_runtime_rs::Message for JoyFeedbackArray {
  type RmwMsg = crate::msg::rmw::JoyFeedbackArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        array: msg.array
          .into_iter()
          .map(|elem| crate::msg::JoyFeedback::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        array: msg.array
          .iter()
          .map(|elem| crate::msg::JoyFeedback::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      array: msg.array
          .into_iter()
          .map(crate::msg::JoyFeedback::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserEcho {
    pub echoes: Vec<f32>,
}



impl Default for LaserEcho {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::LaserEcho::default())
  }
}

impl rosidl_runtime_rs::Message for LaserEcho {
  type RmwMsg = crate::msg::rmw::LaserEcho;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        echoes: msg.echoes.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        echoes: msg.echoes.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      echoes: msg.echoes
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserScan {
    pub header: std_msgs::msg::Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}



impl Default for LaserScan {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::LaserScan::default())
  }
}

impl rosidl_runtime_rs::Message for LaserScan {
  type RmwMsg = crate::msg::rmw::LaserScan;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        angle_min: msg.angle_min,
        angle_max: msg.angle_max,
        angle_increment: msg.angle_increment,
        time_increment: msg.time_increment,
        scan_time: msg.scan_time,
        range_min: msg.range_min,
        range_max: msg.range_max,
        ranges: msg.ranges.into(),
        intensities: msg.intensities.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      angle_min: msg.angle_min,
      angle_max: msg.angle_max,
      angle_increment: msg.angle_increment,
      time_increment: msg.time_increment,
      scan_time: msg.scan_time,
      range_min: msg.range_min,
      range_max: msg.range_max,
        ranges: msg.ranges.as_slice().into(),
        intensities: msg.intensities.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      angle_min: msg.angle_min,
      angle_max: msg.angle_max,
      angle_increment: msg.angle_increment,
      time_increment: msg.time_increment,
      scan_time: msg.scan_time,
      range_min: msg.range_min,
      range_max: msg.range_max,
      ranges: msg.ranges
          .into_iter()
          .collect(),
      intensities: msg.intensities
          .into_iter()
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MagneticField {
    pub header: std_msgs::msg::Header,
    pub magnetic_field: geometry_msgs::msg::Vector3,
    pub magnetic_field_covariance: [f64; 9],
}



impl Default for MagneticField {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MagneticField::default())
  }
}

impl rosidl_runtime_rs::Message for MagneticField {
  type RmwMsg = crate::msg::rmw::MagneticField;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        magnetic_field: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.magnetic_field)).into_owned(),
        magnetic_field_covariance: msg.magnetic_field_covariance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        magnetic_field: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.magnetic_field)).into_owned(),
        magnetic_field_covariance: msg.magnetic_field_covariance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      magnetic_field: geometry_msgs::msg::Vector3::from_rmw_message(msg.magnetic_field),
      magnetic_field_covariance: msg.magnetic_field_covariance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointState {
    pub header: std_msgs::msg::Header,
    pub joint_names: Vec<std::string::String>,
    pub transforms: Vec<geometry_msgs::msg::Transform>,
    pub twist: Vec<geometry_msgs::msg::Twist>,
    pub wrench: Vec<geometry_msgs::msg::Wrench>,
}



impl Default for MultiDOFJointState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MultiDOFJointState::default())
  }
}

impl rosidl_runtime_rs::Message for MultiDOFJointState {
  type RmwMsg = crate::msg::rmw::MultiDOFJointState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        transforms: msg.transforms
          .into_iter()
          .map(|elem| geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        twist: msg.twist
          .into_iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        wrench: msg.wrench
          .into_iter()
          .map(|elem| geometry_msgs::msg::Wrench::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        joint_names: msg.joint_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        transforms: msg.transforms
          .iter()
          .map(|elem| geometry_msgs::msg::Transform::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        twist: msg.twist
          .iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        wrench: msg.wrench
          .iter()
          .map(|elem| geometry_msgs::msg::Wrench::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
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
      transforms: msg.transforms
          .into_iter()
          .map(geometry_msgs::msg::Transform::from_rmw_message)
          .collect(),
      twist: msg.twist
          .into_iter()
          .map(geometry_msgs::msg::Twist::from_rmw_message)
          .collect(),
      wrench: msg.wrench
          .into_iter()
          .map(geometry_msgs::msg::Wrench::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiEchoLaserScan {
    pub header: std_msgs::msg::Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<crate::msg::LaserEcho>,
    pub intensities: Vec<crate::msg::LaserEcho>,
}



impl Default for MultiEchoLaserScan {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::MultiEchoLaserScan::default())
  }
}

impl rosidl_runtime_rs::Message for MultiEchoLaserScan {
  type RmwMsg = crate::msg::rmw::MultiEchoLaserScan;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        angle_min: msg.angle_min,
        angle_max: msg.angle_max,
        angle_increment: msg.angle_increment,
        time_increment: msg.time_increment,
        scan_time: msg.scan_time,
        range_min: msg.range_min,
        range_max: msg.range_max,
        ranges: msg.ranges
          .into_iter()
          .map(|elem| crate::msg::LaserEcho::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        intensities: msg.intensities
          .into_iter()
          .map(|elem| crate::msg::LaserEcho::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      angle_min: msg.angle_min,
      angle_max: msg.angle_max,
      angle_increment: msg.angle_increment,
      time_increment: msg.time_increment,
      scan_time: msg.scan_time,
      range_min: msg.range_min,
      range_max: msg.range_max,
        ranges: msg.ranges
          .iter()
          .map(|elem| crate::msg::LaserEcho::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        intensities: msg.intensities
          .iter()
          .map(|elem| crate::msg::LaserEcho::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      angle_min: msg.angle_min,
      angle_max: msg.angle_max,
      angle_increment: msg.angle_increment,
      time_increment: msg.time_increment,
      scan_time: msg.scan_time,
      range_min: msg.range_min,
      range_max: msg.range_max,
      ranges: msg.ranges
          .into_iter()
          .map(crate::msg::LaserEcho::from_rmw_message)
          .collect(),
      intensities: msg.intensities
          .into_iter()
          .map(crate::msg::LaserEcho::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatFix {
    pub header: std_msgs::msg::Header,
    pub status: crate::msg::NavSatStatus,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}

impl NavSatFix {
    /// If the covariance of the fix is known, fill it in completely. If the
    /// GPS receiver provides the variance of each measurement, put them
    /// along the diagonal. If only Dilution of Precision is available,
    /// estimate an approximate covariance from that.
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}


impl Default for NavSatFix {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::NavSatFix::default())
  }
}

impl rosidl_runtime_rs::Message for NavSatFix {
  type RmwMsg = crate::msg::rmw::NavSatFix;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        status: crate::msg::NavSatStatus::into_rmw_message(std::borrow::Cow::Owned(msg.status)).into_owned(),
        latitude: msg.latitude,
        longitude: msg.longitude,
        altitude: msg.altitude,
        position_covariance: msg.position_covariance,
        position_covariance_type: msg.position_covariance_type,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        status: crate::msg::NavSatStatus::into_rmw_message(std::borrow::Cow::Borrowed(&msg.status)).into_owned(),
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
        position_covariance: msg.position_covariance,
      position_covariance_type: msg.position_covariance_type,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      status: crate::msg::NavSatStatus::from_rmw_message(msg.status),
      latitude: msg.latitude,
      longitude: msg.longitude,
      altitude: msg.altitude,
      position_covariance: msg.position_covariance,
      position_covariance_type: msg.position_covariance_type,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatStatus {
    pub status: i8,
    pub service: u16,
}

impl NavSatStatus {
    /// status is not yet set
    pub const STATUS_UNKNOWN: i8 = -2;
    /// unable to fix position
    pub const STATUS_NO_FIX: i8 = -1;
    /// unaugmented fix
    pub const STATUS_FIX: i8 = 0;
    /// with satellite-based augmentation
    pub const STATUS_SBAS_FIX: i8 = 1;
    /// with ground-based augmentation
    pub const STATUS_GBAS_FIX: i8 = 2;
    /// Bits defining which Global Navigation Satellite System signals were
    /// used by the receiver.
    /// Remember service is a bitfield, so checking (service & SERVICE_UNKNOWN) will not work. Use == instead.
    pub const SERVICE_UNKNOWN: u16 = 0;
    pub const SERVICE_GPS: u16 = 1;
    pub const SERVICE_GLONASS: u16 = 2;
    /// includes BeiDou.
    pub const SERVICE_COMPASS: u16 = 4;
    pub const SERVICE_GALILEO: u16 = 8;
}


impl Default for NavSatStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::NavSatStatus::default())
  }
}

impl rosidl_runtime_rs::Message for NavSatStatus {
  type RmwMsg = crate::msg::rmw::NavSatStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        service: msg.service,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      service: msg.service,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      service: msg.service,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointCloud {
    pub header: std_msgs::msg::Header,
    pub points: Vec<geometry_msgs::msg::Point32>,
    pub channels: Vec<crate::msg::ChannelFloat32>,
}



impl Default for PointCloud {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::PointCloud::default())
  }
}

impl rosidl_runtime_rs::Message for PointCloud {
  type RmwMsg = crate::msg::rmw::PointCloud;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point32::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        channels: msg.channels
          .into_iter()
          .map(|elem| crate::msg::ChannelFloat32::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| geometry_msgs::msg::Point32::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        channels: msg.channels
          .iter()
          .map(|elem| crate::msg::ChannelFloat32::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      points: msg.points
          .into_iter()
          .map(geometry_msgs::msg::Point32::from_rmw_message)
          .collect(),
      channels: msg.channels
          .into_iter()
          .map(crate::msg::ChannelFloat32::from_rmw_message)
          .collect(),
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointCloud2 {
    pub header: std_msgs::msg::Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<crate::msg::PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}



impl Default for PointCloud2 {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::PointCloud2::default())
  }
}

impl rosidl_runtime_rs::Message for PointCloud2 {
  type RmwMsg = crate::msg::rmw::PointCloud2;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        height: msg.height,
        width: msg.width,
        fields: msg.fields
          .into_iter()
          .map(|elem| crate::msg::PointField::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        is_bigendian: msg.is_bigendian,
        point_step: msg.point_step,
        row_step: msg.row_step,
        data: msg.data.into(),
        is_dense: msg.is_dense,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      height: msg.height,
      width: msg.width,
        fields: msg.fields
          .iter()
          .map(|elem| crate::msg::PointField::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      is_bigendian: msg.is_bigendian,
      point_step: msg.point_step,
      row_step: msg.row_step,
        data: msg.data.as_slice().into(),
      is_dense: msg.is_dense,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      height: msg.height,
      width: msg.width,
      fields: msg.fields
          .into_iter()
          .map(crate::msg::PointField::from_rmw_message)
          .collect(),
      is_bigendian: msg.is_bigendian,
      point_step: msg.point_step,
      row_step: msg.row_step,
      data: msg.data
          .into_iter()
          .collect(),
      is_dense: msg.is_dense,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointField {
    pub name: std::string::String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

impl PointField {
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const INT32: u8 = 5;
    pub const UINT32: u8 = 6;
    pub const FLOAT32: u8 = 7;
    pub const FLOAT64: u8 = 8;
    pub const INT64: u8 = 9;
    pub const UINT64: u8 = 10;
    pub const BOOL: u8 = 11;
}


impl Default for PointField {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::PointField::default())
  }
}

impl rosidl_runtime_rs::Message for PointField {
  type RmwMsg = crate::msg::rmw::PointField;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        offset: msg.offset,
        datatype: msg.datatype,
        count: msg.count,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      offset: msg.offset,
      datatype: msg.datatype,
      count: msg.count,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      offset: msg.offset,
      datatype: msg.datatype,
      count: msg.count,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Range {
    pub header: std_msgs::msg::Header,
    pub radiation_type: u8,
    pub field_of_view: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range: f32,
    pub variance: f32,
}

impl Range {
    /// Radiation type enums
    /// If you want a value added to this list, send an email to the ros-users list
    pub const ULTRASOUND: u8 = 0;
    pub const INFRARED: u8 = 1;
}


impl Default for Range {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Range::default())
  }
}

impl rosidl_runtime_rs::Message for Range {
  type RmwMsg = crate::msg::rmw::Range;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        radiation_type: msg.radiation_type,
        field_of_view: msg.field_of_view,
        min_range: msg.min_range,
        max_range: msg.max_range,
        range: msg.range,
        variance: msg.variance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      radiation_type: msg.radiation_type,
      field_of_view: msg.field_of_view,
      min_range: msg.min_range,
      max_range: msg.max_range,
      range: msg.range,
      variance: msg.variance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      radiation_type: msg.radiation_type,
      field_of_view: msg.field_of_view,
      min_range: msg.min_range,
      max_range: msg.max_range,
      range: msg.range,
      variance: msg.variance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}



impl Default for RegionOfInterest {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::RegionOfInterest::default())
  }
}

impl rosidl_runtime_rs::Message for RegionOfInterest {
  type RmwMsg = crate::msg::rmw::RegionOfInterest;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x_offset: msg.x_offset,
        y_offset: msg.y_offset,
        height: msg.height,
        width: msg.width,
        do_rectify: msg.do_rectify,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x_offset: msg.x_offset,
      y_offset: msg.y_offset,
      height: msg.height,
      width: msg.width,
      do_rectify: msg.do_rectify,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x_offset: msg.x_offset,
      y_offset: msg.y_offset,
      height: msg.height,
      width: msg.width,
      do_rectify: msg.do_rectify,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RelativeHumidity {
    pub header: std_msgs::msg::Header,
    pub relative_humidity: f64,
    pub variance: f64,
}



impl Default for RelativeHumidity {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::RelativeHumidity::default())
  }
}

impl rosidl_runtime_rs::Message for RelativeHumidity {
  type RmwMsg = crate::msg::rmw::RelativeHumidity;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        relative_humidity: msg.relative_humidity,
        variance: msg.variance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      relative_humidity: msg.relative_humidity,
      variance: msg.variance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      relative_humidity: msg.relative_humidity,
      variance: msg.variance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Temperature {
    pub header: std_msgs::msg::Header,
    pub temperature: f64,
    pub variance: f64,
}



impl Default for Temperature {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Temperature::default())
  }
}

impl rosidl_runtime_rs::Message for Temperature {
  type RmwMsg = crate::msg::rmw::Temperature;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        temperature: msg.temperature,
        variance: msg.variance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      temperature: msg.temperature,
      variance: msg.variance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      temperature: msg.temperature,
      variance: msg.variance,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TimeReference {
    pub header: std_msgs::msg::Header,
    pub time_ref: builtin_interfaces::msg::Time,
    pub source: std::string::String,
}



impl Default for TimeReference {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::TimeReference::default())
  }
}

impl rosidl_runtime_rs::Message for TimeReference {
  type RmwMsg = crate::msg::rmw::TimeReference;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        time_ref: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.time_ref)).into_owned(),
        source: msg.source.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        time_ref: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_ref)).into_owned(),
        source: msg.source.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      time_ref: builtin_interfaces::msg::Time::from_rmw_message(msg.time_ref),
      source: msg.source.to_string(),
    }
  }
}


