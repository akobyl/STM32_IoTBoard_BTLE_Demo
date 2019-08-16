# STM32 Bluetooth Low Energy Demo #

## Hardware ##

L475E_IOT01

## Development Software ##

This project has been configured to work with TrueStudio 9.3.  The project was initilaized with STM32CubeMx, but it is no longer compatible with future STM32CubeMx updates, so make any changes with care!

## Using STM HAL firmware ##

### BTLE API Usage ###

ST's Guide can be found here: [Getting started with the X-CUBE-BLE1 Bluetooth Low Energy software expansion for STM32Cube](https://www.st.com/content/ccc/resource/technical/document/user_manual/b3/37/62/19/ea/9f/48/4d/DM00169392.pdf/files/DM00169392.pdf/jcr:content/translations/en.DM00169392.pdf)


#### BTLE Initialization ####

Initialize Hardware Abstraction Layer and serial interface:

* SPI serial initialization:
  * `BNRG_SPI_Init();` or
  * 
* `HCI_Init()`
* `BlueNRG_RST();`

Initialize services and characterisitics:

* `aci_gap_init()`
* `aci_gatt_add_serv()`

Set security requirements:

* `aci_gap_set_auth_requirement()`

Set discoverable mode:

* `aci_gap_set_discoverable()`

#### BTLE Files ####

* `gatt_db.c`: Sets up the services and includes the service UUIDs as definitions
* 


### Sensor API Usage ###

ST's Guide can be found here: [Getting started with the X-CUBE-MEMS1 motion MEMS and environmental
sensor software expansion for STM32Cube](https://www.st.com/content/ccc/resource/technical/document/user_manual/c8/04/7f/d1/e9/24/47/09/DM00157069.pdf/files/DM00157069.pdf/jcr:content/translations/en.DM00157069.pdf)