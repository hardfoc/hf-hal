# TMC9660Handler Architecture Update

## Overview
The TMC9660Handler has been completely refactored to accept BaseSpi and/or BaseUart interfaces instead of requiring a pre-constructed TMC9660CommInterface. This provides better flexibility and integration with the existing HardFOC communication architecture.

## Key Changes

### 1. New Communication Interface Architecture
- **Added `Tmc9660SpiCommInterface`**: Bridges BaseSpi to SPITMC9660CommInterface
- **Added `Tmc9660UartCommInterface`**: Bridges BaseUart to UARTTMC9660CommInterface
- **Internal Communication Management**: TMC9660Handler creates and manages the appropriate communication interfaces internally

### 2. Updated Constructor Options
The TMC9660Handler now supports three constructor variants:

```cpp
// SPI-only communication
Tmc9660Handler(BaseSpi& spi_interface, uint8_t address, 
               const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

// UART-only communication  
Tmc9660Handler(BaseUart& uart_interface, uint8_t address,
               const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

// Dual interface (SPI takes precedence)
Tmc9660Handler(BaseSpi& spi_interface, BaseUart& uart_interface, uint8_t address,
               const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);
```

### 3. New Interface Management Methods
- **`GetCommMode()`**: Returns current communication mode (SPI or UART)
- **`HasSpiInterface()`**: Checks if SPI interface is available
- **`HasUartInterface()`**: Checks if UART interface is available  
- **`SwitchCommInterface(CommMode mode)`**: Switches between available interfaces

### 4. Communication Interface Implementation Details

#### Tmc9660SpiCommInterface
- Implements `SPITMC9660CommInterface::spiTransfer()`
- Uses `BaseSpi::Transfer()` method for 8-byte full-duplex communication
- Automatically ensures SPI initialization before transfers

#### Tmc9660UartCommInterface  
- Implements `UARTTMC9660CommInterface::sendUartDatagram()` and `receiveUartDatagram()`
- Uses `BaseUart::Write()` and `BaseUart::Read()` methods for 9-byte communication
- Includes timeout handling for UART reception (1 second default)

### 5. Integration with CommChannelsManager
The updated architecture integrates seamlessly with the existing CommChannelsManager:

```cpp
// Get communication interfaces from CommChannelsManager
CommChannelsManager& commManager = CommChannelsManager::GetInstance();
BaseSpi* spiDevice = commManager.GetSpiDevice(SpiDeviceId::TMC9660_SPI);
BaseUart& uartDevice = commManager.GetUart(0);

// Create TMC9660Handler with retrieved interfaces
Tmc9660Handler tmc9660Handler(*spiDevice, uartDevice, TMC9660_ADDRESS);
```

## Benefits

### 1. **Cleaner Integration**
- No need to manually create TMC9660CommInterface instances
- Direct integration with BaseSpi/BaseUart from CommChannelsManager
- Follows the same pattern as other handlers in the system

### 2. **Flexible Communication**
- Support for SPI-only, UART-only, or dual-interface configurations
- Runtime switching between communication methods when both are available
- Automatic fallback to available interface

### 3. **Better Resource Management**
- Communication interfaces are owned and managed by TMC9660Handler
- Automatic initialization and cleanup of communication resources
- Thread-safe interface switching

### 4. **Consistent API**
- Same GPIO and ADC wrapper interfaces as before
- Maintains compatibility with existing BaseGpio and BaseAdc usage
- No changes required to existing GPIO/ADC access code

## Usage Examples

### Basic SPI Usage
```cpp
BaseSpi* spiDevice = commManager.GetSpiDevice(0);
Tmc9660Handler handler(*spiDevice, 0x01);
handler.Initialize();

// Access GPIO and ADC as before
auto& gpio17 = handler.gpio(17);
auto& adc = handler.adc();
```

### Dual Interface with Switching
```cpp
BaseSpi* spiDevice = commManager.GetSpiDevice(0);
BaseUart& uartDevice = commManager.GetUart(0);
Tmc9660Handler handler(*spiDevice, uartDevice, 0x01);

// Start with SPI (default)
handler.Initialize();

// Switch to UART for specific operations
handler.SwitchCommInterface(CommMode::UART);
// ... perform UART operations ...

// Switch back to SPI
handler.SwitchCommInterface(CommMode::SPI);
```

## Migration Guide

### Old Architecture (Before)
```cpp
// Manual TMC9660CommInterface creation required
TMC9660CommInterface& commInterface = /* manually created */;
Tmc9660Handler handler(commInterface, address, bootConfig);
```

### New Architecture (After)  
```cpp
// Direct BaseSpi/BaseUart usage
BaseSpi* spi = commManager.GetSpiDevice(0);
Tmc9660Handler handler(*spi, address, bootConfig);
```

The new architecture eliminates the need for manual communication interface management while providing greater flexibility and better integration with the existing HardFOC communication system.
