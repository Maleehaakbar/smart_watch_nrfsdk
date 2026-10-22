set(USB_DIR ${PROJECT_SOURCE_DIR}/src/app/usb_common)

target_include_directories(app PRIVATE ${USB_DIR})
target_sources_ifdef(CONFIG_USB_DEVICE_STACK_NEXT app PRIVATE
  ${USB_DIR}/usb_init.c
)