

#include "platform_compat.h"
#include <driver/uart.h>
#include "UTILITIES/common/RtosCompat.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdio>

#include "UTILITIES/common/RtosCompat.h"
#include <HAL/component_handlers/ConsolePort.h>

#include "UTILITIES/common/Mutex.h"
#include "UTILITIES/common/MutexGuard.h"
#include <UTILITIES/common/TxUtility.h>
#include <UTILITIES/common/CrcCalculator.h>
#include <UTILITIES/common/SoftwareVersion.h>


//==============================================================//
/// VERBOSE??
//==============================================================//

static bool verbose_console_command_send = true;

//==============================================================//
/// SEMIHOSTING??
//==============================================================//

/// UNCOMMENT IF WANTING TO SEMIHOST IN ORDER TO PRINTF OVER JTAG HEADER.
//#define SEMI_HOSTING

#ifdef SEMI_HOSTING
#ifdef __GNUC__   /* GCC Compiler */
	extern "C" void initialise_monitor_handles(void);
#endif
#endif

//==============================================================//
/// STRINGS
//==============================================================//

static constexpr const char Leader[] = "\r\n   ";
static constexpr const char NewLine[] = "\r\n";

const char ConsolePort::txMutexName[] = "ConsoleTxMutex";
const char ConsolePort::rxMutexName[] = "ConsoleRxMutex";
const char ConsolePort::eventFlagsName[] = "ConsolePortEventFlagsGroup";

//==============================================================//
//==============================================================//

// ------------------------------------ Definitions of 'C-callable' functions --------------------------------------------------

extern "C"
{
    void ConsoleWrite(const char* format, ...)
    {
        if (ConsolePort::GetInstance().outputEnabled)
        {
            if (ConsolePort::GetInstance().EnsureInitialized())
            {
                va_list argumentList;
                va_start(argumentList, format);

                /// Call the static Write method of ConsolePort
                ConsolePort::WriteSmallBuffer(format, argumentList);

                va_end(argumentList);
            }
        }
    }
}

// ------------------------------------ End of definitions of 'C-callable' functions --------------------------------------------------

//----------------------------------------------- Native c++ code ----------------------------------------------------------------------

ConsolePort& ConsolePort::GetInstance() noexcept
{
         static ConsolePort consolePort(UART_NUM_0);
	 return consolePort;
}

///-----------------------------------------------------------------------------------------------
/// <summary>
/// Initializes a new instance of class DebugPort with the assigned pins.  The first instance created is considered the 
/// </summary>
/// <param name="usart">Assigned USART</param>
/// <param name="baudRate">Desired baud rate</param>


ConsolePort::ConsolePort(uart_port_t txUartArg)  noexcept :
    txUart(txUartArg),
	inputEnabled( true ),
    outputEnabled( true ),
    initialized( false ),
	readData(),
	receivedChar(),
	txMutex(txMutexName),
	rxMutex(rxMutexName),
	txMutexExternalLock( false ),
	rxMutexExternalLock( false ),
	currentLogLevel(DEBUG),
	lastWriteTime(0),
    txBuffer(),
    rxBuffer(),
	consoleEventFlags{},
    consoleEventFlagsCreated(false),
	isTransmitBufferEmptiedFlagSet(true)
{
	/// No code at this time.
}

ConsolePort::~ConsolePort() noexcept
{
    /// Delete the event flag created
    if(consoleEventFlagsCreated){
        tx_event_flags_delete(&consoleEventFlags);
    }
}

bool ConsolePort::Initialize() noexcept
{
    //=============================================//
    /// ENABLE SEMIHOSTING
    //=============================================//

    /**< enable Semi-hosting to print errors while debugging */
    #ifdef SEMI_HOSTING
      #ifdef __GNUC__
          if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
          {
              initialise_monitor_handles();
          }
      #endif
    #endif

	bool success = false;
	MutexGuard txGuard(txMutex, MutexGuard::MaxInitializationTimeMsec, &success);  /// Create the mutex and lock it, check for success
	if( success )
	{
		MutexGuard rxGuard(rxMutex, MutexGuard::MaxInitializationTimeMsec, &success);  /// Create the mutex and lock it, check for success
		if( success )
		{
			if(!consoleEventFlagsCreated) {
				/** Create the event flags group. */
				UINT status = tx_event_flags_create(&consoleEventFlags, (CHAR *)(eventFlagsName));

				 if(status == TX_SUCCESS){
					 consoleEventFlagsCreated = true;
				 }
			}

                    uart_config_t cfg = {
                        .baud_rate = 115200,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .source_clk = UART_SCLK_DEFAULT
                    };
                    esp_err_t openResult = uart_param_config(txUart, &cfg);
                    if (openResult == ESP_OK) {
                        openResult = uart_driver_install(txUart, 256, 0, 0, nullptr, 0);
                    }

		    /// Initiate Read (Only use if wanting to receive characters with the
		    /// UART_EVENT_RX_COMPLETE flag. However, since we're using the transfer instance
		    /// as defined under the uart block in the configuration.xml, whenever we receive
		    /// a character that is not requested for, it will be received under the Interrupt handler
		    /// with the UART_EVENT_RX_CHAR event so no real need for this.
		    ///
		    /// However if we want to go to the mode of requesting a number of bytes, we can always follow this route.
                    //uart_read_bytes(txUart, receivedChar, 1, portMAX_DELAY);

                    return( openResult == ESP_OK || openResult == ESP_ERR_INVALID_STATE);
		}
	}
	return false;
}

//========================================================//
/// SEMIHOST TERMINAL WRITERS
//========================================================//

void ConsolePort::WriteSemihost(const char* format, ...) noexcept {
#ifdef SEMI_HOSTING
    va_list args;
    va_start(args, format);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, true);
    va_end(args);
#else
    UTIL_UNUSED(format);
#endif
}

//========================================================//
/// SERIAL CONSOLE WRITERS
//========================================================//

//==================================//
/// ENABLERS
//==================================//

bool ConsolePort::EnableOutput() noexcept {
	if(!EnsureInitialized()) { return false; }

	bool success = false;
	MutexGuard guard(txMutex, TX_NO_WAIT, &success);
	if(!success) { return false; }

	outputEnabled = true;
	return true;
}

bool ConsolePort::DisableOutput() noexcept {
	if(!EnsureInitialized()) { return false; }

	bool success = false;
	MutexGuard guard(txMutex, TX_NO_WAIT, &success);
	if(!success) { return false; }

	outputEnabled = false;
	return true;
}

bool ConsolePort::EnableInput() noexcept {
	if(!EnsureInitialized()) { return false; }

	bool success = false;
	MutexGuard guard(rxMutex, TX_NO_WAIT, &success);
	if(!success) { return false; }

	inputEnabled = true;
	return true;
}

bool ConsolePort::DisableInput() noexcept {
	if(!EnsureInitialized()) { return false; }

	bool success = false;
	MutexGuard guard(rxMutex, TX_NO_WAIT, &success);
	if(!success) { return false; }

	inputEnabled = false;
	return true;
}

//==================================//
/// RECEIVED DATA CHECKERS/READERS
//==================================//

bool ConsolePort::IsDataAvailable() noexcept
{
	if(!EnsureInitialized()) { return false; }

    bool empty = rxBuffer.IsEmpty();
    return (empty == false);
}

bool ConsolePort::Read(uint8_t& data) noexcept
{
	/// Read is not blocked if there's data available in received queue even if input is disabled.
	/// A disabled input will stop any more characters from being received and stored.
    if(ConsolePort::IsDataAvailable())
    {
        return (rxBuffer.Read(data));
    }

    return false;
}

bool ConsolePort::WaitForCharReceiveEvent(ULONG wait_option) noexcept
{
	if(!EnsureInitialized()) { return false; }

    /// Wait for specific event flags to be set
    ULONG actual_flags;
    return (TX_SUCCESS == tx_event_flags_get(&consoleEventFlags, consoleReceiveEventFlagPos, TX_AND_CLEAR, &actual_flags, wait_option));
}

bool ConsolePort::WaitForTransmitBufferEmptiedEvent(ULONG wait_option) noexcept
{
	if(!EnsureInitialized()) { return false; }

    /// Wait for specific event flags to be set
    ULONG actual_flags;
    return (TX_SUCCESS == tx_event_flags_get(&consoleEventFlags, consoleTransmitBufferEmptiedFlagPos, TX_AND_CLEAR, &actual_flags, wait_option));
}

//================================//
/// LOGGING LEVEL BASED WRITERS
//================================//

void ConsolePort::SetLogLevel(LogLevel level) {
    GetInstance().currentLogLevel = level;
}

LogLevel ConsolePort::GetLogLevel() {
	return GetInstance().currentLogLevel;
}

void ConsolePort::WriteLog(LogLevel level, const char* format, ...) noexcept {
    if (level < GetInstance().currentLogLevel) {
        return;
    }

    const char* colorCode = GetInstance().GetLogLevelColor(level);
    va_list args;
    va_start(args, format);
    GetInstance().WriteColoredSmallBuffer(format, colorCode, args);
    va_end(args);
}

void ConsolePort::WriteLogConditional(bool verboseFlag, LogLevel level, const char* format, ...) noexcept {
    if (!verboseFlag || level < GetInstance().currentLogLevel) {
        return;
    }

    const char* colorCode = GetInstance().GetLogLevelColor(level);
    va_list args;
    va_start(args, format);
    GetInstance().WriteColoredSmallBuffer(format, colorCode, args);
    va_end(args);
}

//================================//
/// STRAIGHT TO CONSOLE WRITERS
//================================//

// Public function for writing with default behavior
void ConsolePort::Write(const char* format, ...) noexcept {
	LogLevel level = LogLevel::DEBUG;

    if (level < GetInstance().currentLogLevel) {
        return;
    }

    const char* colorCode = GetInstance().GetLogLevelColor(level);
    va_list args;
    va_start(args, format);
    GetInstance().WriteColoredSmallBuffer(format, colorCode, args);
    va_end(args);
}

// Conditional write based on verbosity flag
void ConsolePort::WriteConditional(bool verboseFlag, const char* format, ...) noexcept {
	LogLevel level = LogLevel::DEBUG;

    if (!verboseFlag || level < GetInstance().currentLogLevel) {
        return;
    }

    const char* colorCode = GetInstance().GetLogLevelColor(level);
    va_list args;
    va_start(args, format);
    GetInstance().WriteColoredSmallBuffer(format, colorCode, args);
    va_end(args);
}

// Write with colored line for terminal that support it
void ConsolePort::WriteColored(const char* format, ColorCode colorCode, ...) noexcept {
    va_list args;
    va_start(args, colorCode);
    GetInstance().WriteColoredSmallBuffer(format, GetColorCode(colorCode), args);
    va_end(args);
}

// Write with formatted line for terminal that support it
void ConsolePort::WriteFormatted(const char* format, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteFormattedSmallBuffer(format, GetFormatCode(formatCode), args);
    va_end(args);
}

// Write with colored and formatted line for terminal that support it
void ConsolePort::WriteColoredFormatted(const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteColoredFormattedSmallBuffer(format, GetColorCode(colorCode), GetFormatCode(formatCode), args);
    va_end(args);
}



// Public function for writing without a timestamp
void ConsolePort::WriteNoTimestamp(const char* format, ...) noexcept {
    va_list args;
    va_start(args, format);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(false, true, format, args, true, true, false);
    va_end(args);
}

void ConsolePort::WriteColoredNoTimestamp(const char* format, ColorCode colorCode, ...) noexcept {
    va_list args;
    va_start(args, colorCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(false, true, format, args, true, true, false, GetColorCode(colorCode));
    va_end(args);
}

void ConsolePort::WriteFormattedNoTimestamp(const char* format, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(false, true, format, args, true, true, false, "", GetFormatCode(formatCode));
    va_end(args);
}

void ConsolePort::WriteColoredFormattedNoTimestamp(const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(false, true, format, args, true, true, false, GetColorCode(colorCode), GetFormatCode(formatCode));
    va_end(args);
}



// Public function for writing without newline handling
void ConsolePort::WriteWithNoNewline(bool addTimestamp, const char* format, ...) noexcept {
    va_list args;
    va_start(args, format);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(addTimestamp, false, format, args, true, true, false);
    va_end(args);
}

void ConsolePort::WriteColoredWithNoNewline(bool addTimestamp, const char* format, ColorCode colorCode, ...) noexcept {
    va_list args;
    va_start(args, colorCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(addTimestamp, false, format, args, true, true, false, GetColorCode(colorCode));
    va_end(args);
}

void ConsolePort::WriteFormattedWithNoNewline(bool addTimestamp, const char* format, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(addTimestamp, false, format, args, true, true, false, "", GetFormatCode(formatCode));
    va_end(args);
}

void ConsolePort::WriteColoredFormattedWithNoNewline(bool addTimestamp, const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept {
    va_list args;
    va_start(args, formatCode);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(addTimestamp, false, format, args, true, true, false, GetColorCode(colorCode), GetFormatCode(formatCode));
    va_end(args);
}



// Write messages throttled by a minimum interval
void ConsolePort::WriteThrottled(const char* format, ...) noexcept {
    uint32_t currentTime = GetElapsedTimeMsec();
    if (currentTime - GetInstance().lastWriteTime < 100) {
        return;
    }

    va_list args;
    va_start(args, format);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, false);
    va_end(args);

    GetInstance().lastWriteTime = currentTime;
}

// Write with execution time profiling
void ConsolePort::WriteWithTiming(uint32_t startTime, const char* format, ...) noexcept {
    uint32_t elapsed = GetElapsedTimeMsec() - startTime;

    char buffer[FormatSmallBufferSizeBytes];
    snprintf(buffer, sizeof(buffer), "[Elapsed: %lu ms] %s", elapsed, format);

    va_list args;
    va_start(args, format);
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(false, true, buffer, args, true, true, false);
    va_end(args);
}


// Log JSON-formatted messages
void ConsolePort::WriteJSON(const char* key, const char* value) noexcept {
    char buffer[FormatSmallBufferSizeBytes];
    snprintf(buffer, sizeof(buffer), "{ \"%s\": \"%s\" }\r\n", key, value);
    GetInstance().WriteAppend(buffer, static_cast<uint16_t>(strlen(buffer)));
}

void ConsolePort::NewLine() noexcept
{
    if (GetInstance().outputEnabled)
    {
        if (GetInstance().EnsureInitialized())
        {
            char buffer[3] = "\r\n"; // CR and NL
            GetInstance().WriteAppend(buffer, 2); // Append to output stream and enable interrupts
        }
    }
}

//====================================//
/// SERIAL CONSOLE DATA BLOCK WRITERS
//====================================//

void ConsolePort::WriteDataBlock(const char* format, const void* dataArg, uint32_t dataSizeBytes, ...) noexcept {
    if (!GetInstance().outputEnabled || !GetInstance().EnsureInitialized()) {
        return;
    }

    va_list argumentList;
    va_start(argumentList, dataSizeBytes);

    const uint8_t* data = static_cast<const uint8_t*>(dataArg);
    char buffer[FormatSmallBufferSizeBytes];

    // Add Timestamp and format the initial message
    WriteImplHelper(buffer, sizeof(buffer), true, true, format, argumentList, true, true, false, nullptr, nullptr);

    // Format and write the data as one or more lines of N bytes per line
    uint32_t bytesRemaining = dataSizeBytes;
    bool firstLine = true;

    while (bytesRemaining > 0) {
        // Add the timestamp and blank leader
        va_list emptyArgs;
        va_start(emptyArgs, dataSizeBytes);
        WriteImplHelper(buffer, sizeof(buffer), true, false, "", emptyArgs, false, true, false, nullptr, nullptr);
        va_end(emptyArgs);

        uint16_t bytesToWrite = static_cast<uint16_t>(strlen(buffer));

        if (firstLine) { // First line of data output
            bytesToWrite += static_cast<uint16_t>(snprintf(buffer + bytesToWrite, sizeof(buffer) - bytesToWrite, "(%lu bytes): ", dataSizeBytes));
            firstLine = false;
        } else {
            snprintf(buffer + bytesToWrite, sizeof(buffer) - bytesToWrite, "%s", Leader);
            bytesToWrite += sizeof(Leader) - 1; // Does not include null byte
        }

        while (bytesRemaining > 0 && bytesToWrite < 80) {
            bytesToWrite += static_cast<uint16_t>(snprintf(buffer + bytesToWrite, sizeof(buffer) - bytesToWrite, "0x%02X ", *data++)); // Add a character
            --bytesRemaining;
        }

        // Add CR and NL to end of line if not already there
        if ((buffer[bytesToWrite - 2] != '\r') && (buffer[bytesToWrite - 1] != '\n')) {
            strcat(buffer, "\r\n");
            bytesToWrite += 2U;
        }
        GetInstance().WriteAppend(buffer, bytesToWrite); // Append to output stream and enable interrupts
    }

    va_end(argumentList);
}



//=====================================//
/// SERIAL CONSOLE NOTIFICATION WRITERS
//=====================================//

void ConsolePort::WriteToNotify(const char* input_string, int num_spaces, int num_stars) noexcept {
    std::string star_padding(num_stars, '*');
    std::string space_padding(num_spaces, ' ');

    // Calculate the length of the formatted string
    int length = 10 + 2 * (num_stars + num_spaces) + strlen(input_string);

    // Create the top and bottom strings of '=' characters
    std::string equal_padding(length, '=');

    char formatted_string[FormatSmallBufferSizeBytes]; // Adjust size as needed
    snprintf(formatted_string, sizeof(formatted_string), "===||%s%s%s%s%s||===", star_padding.c_str(), space_padding.c_str(), input_string, space_padding.c_str(), star_padding.c_str());

    NewLine();
    va_list args; // Empty va_list for message case
    GetInstance().WriteNotifySmallBuffer(equal_padding.c_str(), args);
    GetInstance().WriteNotifySmallBuffer(formatted_string, args);
    GetInstance().WriteNotifySmallBuffer(equal_padding.c_str(), args);
    NewLine();
}

void ConsolePort::WriteToNotify(const std::vector<std::string>& input_strings, int num_spaces, int num_stars) noexcept {
    std::string star_padding(num_stars, '*');
    std::string space_padding(num_spaces, ' ');

    // Calculate the maximum length of the formatted strings
    size_t max_length = 0;
    for (const auto& str : input_strings) {
        size_t length = 10 + 2 * (num_stars + num_spaces) + str.length();
        if (length > max_length) {
            max_length = length;
        }
    }

    // Create the top and bottom strings of '=' characters
    std::string equal_padding(max_length, '=');

    NewLine();
    va_list args; // Empty va_list for message case
    GetInstance().WriteNotifySmallBuffer(equal_padding.c_str(), args);

    for (const auto& input_string : input_strings) {
        size_t total_padding = max_length - (10 + 2 * (num_stars + num_spaces) + input_string.length());
        size_t left_padding = total_padding / 2;
        size_t right_padding = total_padding - left_padding;

        std::string left_space_padding(left_padding, ' ');
        std::string right_space_padding(right_padding, ' ');

        char formatted_string[FormatSmallBufferSizeBytes]; // Adjust size as needed
        snprintf(formatted_string, sizeof(formatted_string), "===||%s%s%s%s%s%s%s||===", star_padding.c_str(), space_padding.c_str(), left_space_padding.c_str(), input_string.c_str(), right_space_padding.c_str(), space_padding.c_str(), star_padding.c_str());
        GetInstance().WriteNotifySmallBuffer(formatted_string, args);
    }

    GetInstance().WriteNotifySmallBuffer(equal_padding.c_str(), args);
    NewLine();
}

/**
* @brief Displays a formatted message on the console with customizable padding, border, and side characters.
*
* This function allows you to create attention-grabbing messages on the terminal with various formatting options.
* The message is padded with customizable characters, enclosed within borders, and centered in the display area.
*
* @param message The main content to be displayed.
* @param paddingLength The number of padding characters to use on either side of the message. Default is 10.
* @param paddingChar The character used to pad around the message. Default is '*'.
* @param borderChar The character used for the top and bottom borders. Default is '='.
* @param sideBorderChar The character used for the side borders. Default is '|'.
* @param borderLength The total length of the border line. Default is 75.
* @param verboseInit A boolean flag to determine if the message should be displayed. Default is true.
*
* @example
* // Example usage:
* displayMessage("PRESS POWER BUTTON", 5, '*', '=', '|', 75);
* displayMessage("SYSTEM INITIALIZING", 8, '#', '#', '#', 60);
* displayMessage("UPDATE COMPLETE", 12, '-', '-', '|', 80);
*
* // Example output:
* //
* // =======================================================================
* // |||*****                  PRESS POWER BUTTON                   *****|||
* // =======================================================================
* //
* // ############################################################
* // ############        SYSTEM INITIALIZING         ############
* // ############################################################
* //
* // --------------------------------------------------------------------------------
* // |||------------               UPDATE COMPLETE                    ------------|||
* // --------------------------------------------------------------------------------
*/
void ConsolePort::WriteToNotifyCustom( const std::string& message,
									   int paddingLength,
									   char paddingChar,
									   char borderChar,
									   char sideBorderChar,
									   int borderLength,
									   bool verboseInit ) noexcept
{
    if (verboseInit) {
        NewLine(); // Moves to the next line
        std::string border(borderLength, borderChar);
        std::string padding(paddingLength, paddingChar);
        int totalPadding = borderLength - padding.length() * 2 - message.length() - 6; // Adjusting for side borders
        int sidePadding = totalPadding / 2;

        va_list args; // Empty va_list for message case
        GetInstance().WriteNotifySmallBuffer(border.c_str(), args);
        NewLine();
        std::string concatenated = sideBorderChar + std::string(2, sideBorderChar) + padding +
                                   std::string(sidePadding, ' ') + message +
                                   std::string(sidePadding, ' ') + padding +
                                   std::string(2, sideBorderChar) + sideBorderChar;
        GetInstance().WriteNotifySmallBuffer(concatenated.c_str(), args);
        NewLine();
        GetInstance().WriteNotifySmallBuffer(border.c_str(), args);
        NewLine();
    }
}


void ConsolePort::WriteToNotifyCustom(const std::vector<std::string>& messages,
                                      int paddingLength,
                                      char paddingChar,
                                      char borderChar,
                                      char sideBorderChar,
                                      int borderLength,
                                      bool verboseInit) noexcept
{
    if (verboseInit) {
        NewLine(); // Moves to the next line
        std::string border(borderLength, borderChar);
        std::string padding(paddingLength, paddingChar);

        va_list args; // Empty va_list for message case
        GetInstance().WriteNotifySmallBuffer(border.c_str(), args);
        NewLine();

        for (const auto& message : messages) {
            int totalPadding = borderLength - padding.length() * 2 - message.length() - 6; // Adjusting for side borders
            int sidePadding = totalPadding / 2;

            std::string concatenated = sideBorderChar + std::string(2, sideBorderChar) + padding +
                                       std::string(sidePadding, ' ') + message +
                                       std::string(sidePadding, ' ') + padding +
                                       std::string(2, sideBorderChar) + sideBorderChar;
            GetInstance().WriteNotifySmallBuffer(concatenated.c_str(), args);
            NewLine();
        }

        GetInstance().WriteNotifySmallBuffer(border.c_str(), args);
        NewLine();
    }
}


//========================================================//
/// SPECIALIZED SERIAL CONSOLE WRITERS TO CONSOLE APP
//========================================================//

void ConsolePort::WriteConsoleControl(uint8_t *send_buffer, size_t size, bool log_message) noexcept
{
    if (GetInstance().outputEnabled)
    {
        if (GetInstance().EnsureInitialized())
        {
            /// Assuming CONTROL_MESSAGE_FRAME_SIZE is enough to accommodate '#C' + logging byte + original message + '\r' + '\n'.
            char buffer[CONTROL_MESSAGE_FRAME_SIZE];

            /// Copy #C to the start of the buffer.
            strcpy(buffer, "#C");

            /// Append logging byte based on log_enable
            buffer[2] = log_message ? LOG : NON_LOG;

            /// Copy the send_buffer content to buffer after #C and logging byte
            memcpy(buffer + 3, send_buffer, size);  // +3 is for "#C" and logging byte

            /// Append CR and NL to end of the line
            buffer[size + 3] = '\r';
            buffer[size + 4] = '\n';

            /// The new size to write will be the original size + 3 bytes for '#C' and logging byte + 2 bytes for '\r\n'
            size_t bytesToWrite = size + 5; // +3 for '#C' and logging byte, and +2 for '\r\n'

            /// Write to output stream
            GetInstance().WriteAppend(buffer, static_cast<uint16_t>(bytesToWrite));
        }
    }
}


void ConsolePort::SendToApp(commandFrame frame) noexcept
{
    uint8_t send_buffer[CONTROL_MESSAGE_FRAME_SIZE];

    // Copy the message content to the transmit buffer
    send_buffer[0] = frame.component;
    send_buffer[1] = frame.operation;
    memcpy(&send_buffer[2], frame.data, DATA_SIZE);

    // Calculate and set the CRC value
    uint16_t crc = (uint16_t)crc16((char *)send_buffer, FRAME_DATA_SIZE);
    memcpy(send_buffer+FRAME_DATA_SIZE, (uint8_t *)&crc, FRAME_CRC_SIZE);

    // Write the byte array to the UART port, #C will be appended to it internally
    WriteConsoleControl(send_buffer, CONTROL_MESSAGE_FRAME_SIZE);
}

void ConsolePort::SendToApp(sensorFrame frame) noexcept
{
    uint8_t send_buffer[CONTROL_MESSAGE_FRAME_SIZE];

    // Copy the message content to the transmit buffer
    send_buffer[0] = frame.component;
    send_buffer[1] = frame.operation;
    memcpy(&send_buffer[2], frame.data, MAX_FLOAT_DATA_FIELDS);

    // Calculate and set the CRC value
    uint16_t crc = (uint16_t)crc16((char *)send_buffer, FRAME_DATA_SIZE);
    memcpy(send_buffer+FRAME_DATA_SIZE, (uint8_t *)&crc, FRAME_CRC_SIZE);

    if(verbose_console_command_send) {
    	ConsolePort::GetInstance().WriteDataBlock("ConsolePort::SendToApp - ", send_buffer, CONTROL_MESSAGE_FRAME_SIZE);
    }

//    if(verbose_console_command_send) {
//        // Write the first byte with the timestamp
//        ConsolePort::GetInstance().WriteWithNoNewline(ADD_TIMESTAMP,"0x%02X ", send_buffer[0]);
//
//        // Print the rest of the byte on the same line
//        for (uint8_t i = 1; i < CONTROL_MESSAGE_FRAME_SIZE; i++) {
//            ConsolePort::GetInstance().WriteWithNoNewline(NO_TIMESTAMP,"0x%02X ", send_buffer[i]);
//        }
//
//        // Go to a new line for the next message that will be printed
//        ConsolePort::GetInstance().NewLine();
//        TxDelayMsec(1);
//    }

    // Write the byte array to the UART port, #C will be appended to it internally
    WriteConsoleControl(send_buffer, CONTROL_MESSAGE_FRAME_SIZE);
}

//========================================================//
/// BUFFER APPENDERS
//========================================================//

bool ConsolePort::WriteAppend(const char* data, uint16_t count) noexcept
{
	if (data && (count > 0))
	{
		__disable_irq();

		bool appendedBytes = txBuffer.Write(reinterpret_cast<const uint8_t*>(data), count);

		/// If buffer was emptied, the transmission interrupt callback chain will be stopped.
		/// The next write character write is started in the interrupt handler once last character transmission is done.
		/// However, once buffer empties and there's no characters to send the next write is not started.
		/// Therefore, if we noticed that buffer was empty before starting to write to it, make sure to startup
		/// transfers again.
                bool wasEmpty = isTransmitBufferEmptiedFlagSet;

                if ( wasEmpty )
                {
                        uint8_t dataByte = 0;
                        if (txBuffer.Read(dataByte) )
                        {
                                uart_write_bytes(txUart, (const char*)&dataByte, 1);
                        }
                }
		__enable_irq();
		return appendedBytes;
	}
	return false;
}

bool ConsolePort::ReadAppend(const char* data, uint16_t sizeBytes) noexcept
{
	if (data && (sizeBytes > 0))
	{
		__disable_irq();

		bool appendedBytes = rxBuffer.Write(reinterpret_cast<const uint8_t*>(data), sizeBytes);

		__enable_irq();
		return appendedBytes;
	}
	return false;
}

//========================================================//
/// HELPERS
//========================================================//

/**
 * @brief Get the ANSI escape code for the specified color.
 *
 * @param color The color code enum value.
 * @return const char* The ANSI escape code string for the specified color.
 */
const char* ConsolePort::GetColorCode(ColorCode color) noexcept {
	switch (color) {
		case ColorCode::RED: return "\033[31m";
		case ColorCode::GREEN: return "\033[32m";
		case ColorCode::BLUE: return "\033[34m";
		case ColorCode::YELLOW: return "\033[33m";
		case ColorCode::MAGENTA: return "\033[35m";
		case ColorCode::CYAN: return "\033[36m";
		case ColorCode::WHITE: return "\033[37m";
		case ColorCode::BLACK: return "\033[30m";
		case ColorCode::BRIGHT_RED: return "\033[91m";
		case ColorCode::BRIGHT_GREEN: return "\033[92m";
		case ColorCode::BRIGHT_YELLOW: return "\033[93m";
		case ColorCode::BRIGHT_BLUE: return "\033[94m";
		case ColorCode::BRIGHT_MAGENTA: return "\033[95m";
		case ColorCode::BRIGHT_CYAN: return "\033[96m";
		case ColorCode::BRIGHT_WHITE: return "\033[97m";
		case ColorCode::BOLD_RED: return "\033[1;31m";
		case ColorCode::BOLD_GREEN: return "\033[1;32m";
		case ColorCode::BOLD_BLUE: return "\033[1;34m";
		case ColorCode::BOLD_YELLOW: return "\033[1;33m";
		case ColorCode::BOLD_MAGENTA: return "\033[1;35m";
		case ColorCode::BOLD_CYAN: return "\033[1;36m";
		case ColorCode::BOLD_WHITE: return "\033[1;37m";
		case ColorCode::BOLD_BLACK: return "\033[1;30m";
		case ColorCode::BG_RED: return "\033[41m";
		case ColorCode::BG_GREEN: return "\033[42m";
		case ColorCode::BG_BLUE: return "\033[44m";
		case ColorCode::BG_YELLOW: return "\033[43m";
		case ColorCode::BG_MAGENTA: return "\033[45m";
		case ColorCode::BG_CYAN: return "\033[46m";
		case ColorCode::BG_WHITE: return "\033[47m";
		case ColorCode::BG_BLACK: return "\033[40m";
		case ColorCode::BG_BRIGHT_RED: return "\033[101m";
		case ColorCode::BG_BRIGHT_GREEN: return "\033[102m";
		case ColorCode::BG_BRIGHT_YELLOW: return "\033[103m";
		case ColorCode::BG_BRIGHT_BLUE: return "\033[104m";
		case ColorCode::BG_BRIGHT_MAGENTA: return "\033[105m";
		case ColorCode::BG_BRIGHT_CYAN: return "\033[106m";
		case ColorCode::BG_BRIGHT_WHITE: return "\033[107m";
		case ColorCode::RESET: return "\033[0m";
		default: return "\033[0m";
	}
}

const char* ConsolePort::GetFormatCode(FormatCode format) noexcept {
    switch (format) {
        case FormatCode::ITALIC: return "\033[3m";
        case FormatCode::UNDERLINE: return "\033[4m";
        case FormatCode::STRIKETHROUGH: return "\033[9m";
        case FormatCode::BOLD: return "\033[1m";
        case FormatCode::DIM: return "\033[2m";
        case FormatCode::BLINK: return "\033[5m";
        case FormatCode::REVERSE: return "\033[7m";
        case FormatCode::HIDDEN: return "\033[8m";
        case FormatCode::DOUBLE_UNDERLINE: return "\033[21m";
        case FormatCode::FRAMED: return "\033[51m";
        case FormatCode::ENCIRCLED: return "\033[52m";
        case FormatCode::OVERLINED: return "\033[53m";
        case FormatCode::SUPERSCRIPT: return "\033[73m";
        case FormatCode::SUBSCRIPT: return "\033[74m";
        case FormatCode::RESET: return "\033[0m";
        default: return "\033[0m";
    }
}

std::string ConsolePort::GetFormatCode(const std::vector<FormatCode>& formats) noexcept {
	std::string result;
	for (const auto& format : formats) {
		switch (format) {
			case FormatCode::ITALIC: result += "\033[3m"; break;
			case FormatCode::UNDERLINE: result += "\033[4m"; break;
			case FormatCode::STRIKETHROUGH: result += "\033[9m"; break;
			case FormatCode::BOLD: result += "\033[1m"; break;
			case FormatCode::DIM: result += "\033[2m"; break;
			case FormatCode::BLINK: result += "\033[5m"; break;
			case FormatCode::REVERSE: result += "\033[7m"; break;
			case FormatCode::HIDDEN: result += "\033[8m"; break;
			case FormatCode::DOUBLE_UNDERLINE: result += "\033[21m"; break;
			case FormatCode::FRAMED: result += "\033[51m"; break;
			case FormatCode::ENCIRCLED: result += "\033[52m"; break;
			case FormatCode::OVERLINED: result += "\033[53m"; break;
			case FormatCode::SUPERSCRIPT: result += "\033[73m"; break;
			case FormatCode::SUBSCRIPT: result += "\033[74m"; break;
			case FormatCode::RESET: result += "\033[0m"; break;
			default: result += "\033[0m"; break;
		}
	}
	return result;
}

const char* ConsolePort::GetLogLevelColor(LogLevel level) {
    switch (level) {
        case DEBUG: return GetColorCode(ColorCode::RESET);  // Reset
        case INFO: return GetColorCode(ColorCode::BOLD_MAGENTA);
        case WARN: return GetColorCode(ColorCode::BOLD_YELLOW);
        case ERROR: return GetColorCode(ColorCode::BOLD_RED);
        default: return GetColorCode(ColorCode::RESET);
    }
}

void ConsolePort::WriteSmallBuffer(const char* format, va_list args, const char* colorCode, const char* formatCode) noexcept {
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, false, colorCode, formatCode);
}

/// Write colored messages
void ConsolePort::WriteColoredSmallBuffer(const char* format, const char* colorCode, va_list args) noexcept {
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, false, colorCode, nullptr);
}

/// Write formatted messages
void ConsolePort::WriteFormattedSmallBuffer(const char* format, const char* formatCode, va_list args) noexcept {
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, false, nullptr, formatCode);
}

/// Write colored and formatted messages
void ConsolePort::WriteColoredFormattedSmallBuffer(const char* format, const char* colorCode, const char* formatCode, va_list args) noexcept {
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, true, true, false, colorCode, formatCode);
}

/// Write colored and formatted messages for the Notify function
void ConsolePort::WriteNotifySmallBuffer(const char* format, va_list args) noexcept {
    GetInstance().WriteImpl<FormatSmallBufferSizeBytes>(true, true, format, args, false, true, false, GetColorCode(ColorCode::BRIGHT_CYAN), GetFormatCode(FormatCode::ITALIC));
}

/// Write colored messages
void ConsolePort::WriteImplHelper(char* buffer, size_t bufferSize, bool addTimestamp, bool addNewline, const char* input, va_list args, bool isFormat, bool allowTruncate, bool writeSemihost, const char* colorCode, const char* formatCode) noexcept {
    uint32_t bytesToWrite = snprintf(buffer, bufferSize, "#D ");

    if (bytesToWrite >= bufferSize) {
        if (!allowTruncate) return;
        bytesToWrite = bufferSize - 1;
    }

    if (addTimestamp) {
        unsigned long msec = GetElapsedTimeMsec();
        unsigned long days = msec / (24 * 3600 * 1000);
        msec %= (24 * 3600 * 1000);
        unsigned long hours = msec / (3600 * 1000);
        msec %= (3600 * 1000);
        unsigned long minutes = msec / (60 * 1000);
        msec %= (60 * 1000);
        unsigned long seconds = msec / 1000;
        unsigned long milliseconds = msec % 1000;

        uint32_t timestampBytes = snprintf(buffer + bytesToWrite, bufferSize - bytesToWrite, "%lu.%02lu.%02lu.%02lu.%03lu: ", days, hours, minutes, seconds, milliseconds);
        if (timestampBytes >= bufferSize - bytesToWrite) {
            if (!allowTruncate) return;
            timestampBytes = bufferSize - bytesToWrite - 1;
        }
        bytesToWrite += timestampBytes;
    }

    if (colorCode) {
        uint32_t colorBytes = snprintf(buffer + bytesToWrite, bufferSize - bytesToWrite, "%s", colorCode);
        if (colorBytes >= bufferSize - bytesToWrite) {
            if (!allowTruncate) return;
            colorBytes = bufferSize - bytesToWrite - 1;
        }
        bytesToWrite += colorBytes;
    }

    if (formatCode) {
        uint32_t formatBytes = snprintf(buffer + bytesToWrite, bufferSize - bytesToWrite, "%s", formatCode);
        if (formatBytes >= bufferSize - bytesToWrite) {
            if (!allowTruncate) return;
            formatBytes = bufferSize - bytesToWrite - 1;
        }
        bytesToWrite += formatBytes;
    }

    int remainingSpace = bufferSize - bytesToWrite;
    int formattedBytes;

    if (isFormat) {
        formattedBytes = vsnprintf(buffer + bytesToWrite, remainingSpace, input, args);
    } else {
        formattedBytes = snprintf(buffer + bytesToWrite, remainingSpace, "%s", input);
    }

    if (formattedBytes >= remainingSpace) {
        if (!allowTruncate) return;
        formattedBytes = remainingSpace - 1;
    }

    bytesToWrite += formattedBytes;

    if (colorCode || formatCode) {
        uint32_t resetBytes = snprintf(buffer + bytesToWrite, bufferSize - bytesToWrite, "\033[0m");
        if (resetBytes >= bufferSize - bytesToWrite) {
            if (!allowTruncate) return;
            resetBytes = bufferSize - bytesToWrite - 1;
        }
        bytesToWrite += resetBytes;
    }

    if (addNewline && (bytesToWrite + 3) < bufferSize) { // +3 for \r\n and null terminator
        if ((buffer[bytesToWrite - 2] != '\r') && (buffer[bytesToWrite - 1] != '\n')) {
            strcat(buffer, "\r\n");
            bytesToWrite += 2;
        }
    }

    buffer[bytesToWrite] = '\0';

    if (writeSemihost) {
#ifdef SEMI_HOSTING
        if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == CoreDebug_DHCSR_C_DEBUGEN_Msk) {
            printf("%s", buffer);
        }
#endif
    } else {
        GetInstance().WriteAppend(buffer, static_cast<uint16_t>(bytesToWrite));
        if (addNewline && (buffer[bytesToWrite - 1] != '\n')) {
            NewLine();
        }
    }
}



//========================================================//
/// PROJECT ASCII ART WRITER
//========================================================//

void ConsolePort::WriteAsciiArt(const char* message) noexcept {
    va_list args; /// Empty va_list for message case
    GetInstance().WriteImpl<FormatLargeBufferSizeBytes>(false, true, message, args, true, true, false, GetColorCode(ColorCode::BOLD_CYAN), nullptr);
}

void ConsolePort::PrintProjectAsciiArt() noexcept {
    WriteAsciiArt("||----------------------------------------------------||");
    WriteAsciiArt(CreateArtFromString("Conmed").c_str());
    WriteAsciiArt(CreateArtFromString("Moonshine").c_str());
    WriteAsciiArt(CreateArtFromString(softwareVersion.GetString()).c_str());
    WriteAsciiArt("||----------------------------------------------------||");
}

//========================================================//
/// INTERRUPT HANDLERS
//========================================================//

void ConsolePort::HandleInterrupt(const uart_event_t* event) noexcept
{
    switch (event->type) {
        case UART_DATA: {
            if (inputEnabled) {
                uint8_t data;
                uart_read_bytes(txUart, &data, 1, 0);
                ReadAppend(reinterpret_cast<const char*>(&data), 1);
                tx_event_flags_set(&consoleEventFlags, consoleReceiveEventFlagPos, TX_OR);
            }
            break;
        }
        default: break;
    }
}

//========================================================//
/// CALLBACKS
//========================================================//

void ConsoleTxUartCallback(const uart_event_t* event)
{
   ConsolePort::GetInstance().HandleInterrupt(event);
}

//========================================================//
/// PROJECT ASCII ART CREATOR [LAST FUNCTION DUE TO SIZE]
//========================================================//

// https://lordhypersonic.blogspot.com/2019/02/c-ascii-art-generator.html
std::string ConsolePort::CreateArtFromString( const char* input ) noexcept
{
   std::string artwork;
   const uint32_t stringLength = strlen( input);
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
    	   artwork += "  ___   ";
       if (input[i] == 'B' || input[i] == 'b')
    	   artwork +=" _____  ";
       if (input[i] == 'C' || input[i] == 'c')
    	   artwork +=" _____  ";
       if (input[i] == 'D' || input[i] == 'd')
    	   artwork +=" _____  ";
       if (input[i] == 'E' || input[i] == 'e')
    	   artwork +=" _____  ";
       if (input[i] == 'F' || input[i] == 'f')
    	   artwork +=" _____  ";
       if (input[i] == 'G' || input[i] == 'g')
    	   artwork +=" _____  ";
       if (input[i] == 'H' || input[i] == 'h')
    	   artwork +=" _   _  ";
       if (input[i] == 'I' || input[i] == 'i')
    	   artwork +=" _____  ";
       if (input[i] == 'J' || input[i] == 'j')
    	   artwork +="   ___  ";
       if (input[i] == 'K' || input[i] == 'k')
    	   artwork +=" _   __ ";
       if (input[i] == 'L' || input[i] == 'l')
    	   artwork +=" _      ";
       if (input[i] == 'M' || input[i] == 'm')
    	   artwork +=" __  __  ";
       if (input[i] == 'N' || input[i] == 'n')
    	   artwork +=" _   _  ";
       if (input[i] == 'O' || input[i] == 'o')
    	   artwork +=" _____  ";
       if (input[i] == 'P' || input[i] == 'p')
    	   artwork +=" _____  ";
       if (input[i] == 'Q' || input[i] == 'q')
    	   artwork +=" _____  ";
       if (input[i] == 'R' || input[i] == 'r')
    	   artwork +=" _____  ";
       if (input[i] == 'S' || input[i] == 's')
    	   artwork +=" _____  ";
       if (input[i] == 'T' || input[i] == 't')
    	   artwork +=" _____  ";
       if (input[i] == 'U' || input[i] == 'u')
    	   artwork +=" _   _  ";
       if (input[i] == 'V' || input[i] == 'v')
    	   artwork +=" _   _  ";
       if (input[i] == 'W' || input[i] == 'w')
    	   artwork +=" _    _  ";
       if (input[i] == 'X' || input[i] == 'x')
    	   artwork +="__   __ ";
       if (input[i] == 'Y' || input[i] == 'y')
    	   artwork +="__   __ ";
       if (input[i] == 'Z' || input[i]== 'z')
    	   artwork +=" ______ ";
       if (input[i] == ' ')
    	   artwork +="  ";
       if (input[i] == '`')
           artwork +=" _  ";
       if (input[i] == '~')
           artwork +="      ";
       if (input[i] == '1')
           artwork +=" __   ";
       if (input[i]== '2')
           artwork +=" _____  ";
       if (input[i]== '3')
           artwork +=" _____  ";
       if (input[i] == '4')
           artwork +="   ___  ";
       if (input[i] == '5')
           artwork +=" _____  ";
       if (input[i] == '6')
           artwork +="  ____  ";
       if (input[i] == '7')
           artwork +=" ______ ";
       if(input[i] == '.')
           artwork +="    ";
       if (input[i] == '8')
           artwork +=" _____  ";
       if (input[i] == '9')
           artwork +=" _____  ";
       if (input[i] == '0')
           artwork +=" _____  ";
       if (input[i] == '!')
           artwork +=" _  ";
       if (input[i] == '@')
           artwork +="   ____   ";
       if (input[i] == '#')
           artwork +="   _  _    ";
       if (input[i] == '$')
           artwork +="  _   ";
       if (input[i] == '%')
           artwork +=" _   __ ";
       if (input[i] == '^')
           artwork +=" /\\  ";
       if (input[i] == '&')
           artwork +="         ";
       if (input[i] == '*')
           artwork +="    _     ";
       if (input[i] == '(')
           artwork +="  __ ";
       if (input[i] == ')')
           artwork +="__   ";
       if (input[i] == '-')
           artwork +="         ";
       if (input[i] == '_')
           artwork +="         ";
       if (input[i] == '=')
           artwork +="         ";
       if (input[i] == '+')
           artwork +="        ";
       if (input[i] == '[')
           artwork +=" ___  ";
       if (input[i] == '{')
           artwork +="   __ ";
       if (input[i]== ']')
           artwork +=" ___  ";
       if (input[i] == '}')
           artwork +="__    ";
       if (input[i] == '|')
           artwork +=" _  ";
       if (input[i] == '\\')
           artwork +="__      ";
       if (input[i] == ';')
           artwork +=" _  ";
       if (input[i] == ':')
           artwork +="    ";
       if (input[i] == '\'')
           artwork +=" _  ";
       if (input[i] == '"')
           artwork +=" _ _  ";
       if (input[i] == '<')
           artwork +="   __ ";
       if (input[i] == ',')
           artwork +="    ";
       if (input[i] == '>')
           artwork +="__    ";
       if (input[i] == '/')
           artwork +="     __ ";
       if (input[i] == '?')
           artwork +=" ___   ";
   }
   artwork += Leader;

   //loop will print second layer
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
           artwork +=" / _ \\  ";
       if (input[i] == 'B' || input[i] == 'b')
           artwork +="| ___ \\ ";
       if (input[i] == 'C' || input[i] == 'c')
           artwork +="/  __ \\ ";
       if (input[i] == 'D' || input[i] == 'd')
           artwork +="|  _  \\ ";
       if (input[i] == 'E' || input[i] == 'e')
           artwork +="|  ___| ";
       if (input[i] == 'F' || input[i] == 'f')
           artwork +="|  ___| ";
       if (input[i] == 'G' || input[i] == 'g')
           artwork +="|  __ \\ ";
       if (input[i] == 'H' || input[i] == 'h')
           artwork +="| | | | ";
       if (input[i] == 'I' || input[i] == 'i')
           artwork +="|_   _| ";
       if (input[i] == 'J' || input[i] == 'j')
           artwork +="  |_  | ";
       if (input[i] == 'K' || input[i] == 'k')
           artwork +="| | / / ";
       if (input[i] == 'L' || input[i] == 'l')
           artwork +="| |     ";
       if (input[i] == 'M' || input[i] == 'm')
           artwork +="|  \\/  | ";
       if (input[i] == 'N' || input[i] == 'n')
           artwork +="| \\ | | ";
       if (input[i] == 'O' || input[i] == 'o')
           artwork +="|  _  | ";
       if (input[i] == 'P' || input[i] == 'p')
           artwork +="| ___ \\ ";
       if (input[i] == 'Q' || input[i] == 'q')
           artwork +="|  _  | ";
       if (input[i] == 'R' || input[i] == 'r')
           artwork +="| ___ \\ ";
       if (input[i] == 'S' || input[i] == 's')
           artwork +="/  ___| ";
       if (input[i] == 'T' || input[i] == 't')
           artwork +="|_   _| ";
       if (input[i] == 'U' || input[i] == 'u')
           artwork +="| | | | ";
       if (input[i] == 'V' || input[i] == 'v')
           artwork +="| | | | ";
       if (input[i] == 'W' || input[i] == 'w')
           artwork +="| |  | | ";
       if (input[i] == 'X' || input[i] == 'x')
           artwork +="\\ \\ / / ";
       if (input[i] == 'Y' || input[i] == 'y')
           artwork +="\\ \\ / / ";
       if (input[i] == 'Z' || input[i]== 'z')
           artwork +="|___  / ";
       if (input[i] == ' ')
           artwork +="  ";
       if (input[i] == '`')
           artwork +="( ) ";
       if (input[i] == '~')
           artwork +="      ";
       if (input[i] == '1')
           artwork +="/  |  ";
       if (input[i]== '2')
           artwork +="/ __  \\ ";
       if (input[i]== '3')
           artwork +="|____ | ";
       if (input[i] == '4')
           artwork +="  /   | ";
       if (input[i] == '5')
           artwork +="|  ___| ";
       if (input[i] == '6')
           artwork +=" / ___| ";
       if (input[i] == '7')
           artwork +="|___  / ";
       if(input[i] == '.')
           artwork +="    ";
       if (input[i] == '8')
           artwork +="|  _  | ";
       if (input[i] == '9')
           artwork +="|  _  | ";
       if (input[i] == '0')
           artwork +="|  _  | ";
       if (input[i] == '!')
           artwork +="| | ";
       if (input[i] == '@')
           artwork +="  / __ \\  ";
       if (input[i] == '#')
           artwork +=" _| || |_  ";
       if (input[i] == '$')
           artwork +=" | |  ";
       if (input[i] == '%')
           artwork +="(_) / / ";
       if (input[i] == '^')
           artwork +="|/\\| ";
       if (input[i] == '&')
           artwork +="  ___    ";
       if (input[i] == '*')
           artwork +=" /\\| |/\\  ";
       if (input[i] == '(')
           artwork +=" / / ";
       if (input[i] == ')')
           artwork +="\\ \\  ";
       if (input[i] == '-')
           artwork +="         ";
       if (input[i] == '_')
           artwork +="         ";
       if (input[i] == '=')
           artwork +=" ______  ";
       if (input[i] == '+')
           artwork +="   _    ";
       if (input[i] == '[')
           artwork +="|  _| ";
       if (input[i] == '{')
           artwork +="  / / ";
       if (input[i]== ']')
           artwork +="|_  | ";
       if (input[i] == '}')
           artwork +="\\ \\   ";
       if (input[i] == '|')
           artwork +="| | ";
       if (input[i] == '\\')
           artwork +="\\ \\     ";
       if (input[i] == ';')
           artwork +="(_) ";
       if (input[i] == ':')
           artwork +=" _  ";
       if (input[i] == '\'')
           artwork +="( ) ";
       if (input[i] == '"')
           artwork +="( | ) ";
       if (input[i] == '<')
           artwork +="  / / ";
       if (input[i] == ',')
           artwork +="    ";
       if (input[i] == '>')
           artwork +="\\ \\   ";
       if (input[i] == '/')
           artwork +="    / / ";
       if (input[i] == '?')
           artwork +="|__ \\  ";
   }
   artwork += Leader;
   //loop will print third layer
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
           artwork +="/ /_\\ \\ ";
       if (input[i] == 'B' || input[i] == 'b')
           artwork +="| |_/ / ";
       if (input[i] == 'C' || input[i] == 'c')
           artwork +="| /  \\/ ";
       if (input[i] == 'D' || input[i] == 'd')
           artwork +="| | | | ";
       if (input[i] == 'E' || input[i] == 'e')
           artwork +="| |__   ";
       if (input[i] == 'F' || input[i] == 'f')
           artwork +="| |_    ";
       if (input[i] == 'G' || input[i] == 'g')
           artwork +="| |  \\/ ";
       if (input[i] == 'H' || input[i] == 'h')
           artwork +="| |_| | ";
       if (input[i] == 'I' || input[i] == 'i')
           artwork +="  | |   ";
       if (input[i] == 'J' || input[i] == 'j')
           artwork +="    | | ";
       if (input[i] == 'K' || input[i] == 'k')
           artwork +="| |/ /  ";
       if (input[i] == 'L' || input[i] == 'l')
           artwork +="| |     ";
       if (input[i] == 'M' || input[i] == 'm')
           artwork +="| .  . | ";
       if (input[i] == 'N' || input[i] == 'n')
           artwork +="|  \\| | ";
       if (input[i] == 'O' || input[i] == 'o')
           artwork +="| | | | ";
       if (input[i] == 'P' || input[i] == 'p')
           artwork +="| |_/ / ";
       if (input[i] == 'Q' || input[i] == 'q')
           artwork +="| | | | ";
       if (input[i] == 'R' || input[i] == 'r')
           artwork +="| |_/ / ";
       if (input[i] == 'S' || input[i] == 's')
           artwork +="\\ `--.  ";
       if (input[i] == 'T' || input[i] == 't')
           artwork +="  | |   ";
       if (input[i] == 'U' || input[i] == 'u')
           artwork +="| | | | ";
       if (input[i] == 'V' || input[i] == 'v')
           artwork +="| | | | ";
       if (input[i] == 'W' || input[i] == 'w')
           artwork +="| |  | | ";
       if (input[i] == 'X' || input[i] == 'x')
           artwork +=" \\ V /  ";
       if (input[i] == 'Y' || input[i] == 'y')
           artwork +=" \\ V /  ";
       if (input[i] == 'Z' || input[i]== 'z')
           artwork +="   / /  ";
       if (input[i] == ' ')
           artwork +="  ";
       if (input[i] == '`')
           artwork +=" \\| ";
       if (input[i] == '~')
           artwork +=" /\\/| ";
       if (input[i] == '1')
           artwork +="`| |  ";
       if (input[i]== '2')
           artwork +="`' / /' ";
       if (input[i]== '3')
           artwork +="    / / ";
       if (input[i] == '4')
           artwork +=" / /| | ";
       if (input[i] == '5')
           artwork +="|___ \\  ";
       if (input[i] == '6')
           artwork +="/ /___  ";
       if (input[i] == '7')
           artwork +="   / /  ";
       if(input[i] == '.')
           artwork +="    ";
       if (input[i] == '8')
           artwork +=" \\ V /  ";
       if (input[i] == '9')
           artwork +="| |_| | ";
       if (input[i] == '0')
           artwork +="| |/' | ";
       if (input[i] == '!')
           artwork +="| | ";
       if (input[i] == '@')
           artwork +=" / / _` | ";
       if (input[i] == '#')
           artwork +="|_  __  _| ";
       if (input[i] == '$')
           artwork +="/ __) ";
       if (input[i] == '%')
           artwork +="   / /  ";
       if (input[i] == '^')
           artwork +="     ";
       if (input[i] == '&')
           artwork +=" ( _ )   ";
       if (input[i] == '*')
           artwork +=" \\ ` ' /  ";
       if (input[i] == '(')
           artwork +="| |  ";
       if (input[i] == ')')
           artwork +=" | | ";
       if (input[i] == '-')
           artwork +=" ______  ";
       if (input[i] == '_')
           artwork +="         ";
       if (input[i] == '=')
           artwork +="|______| ";
       if (input[i] == '+')
           artwork +=" _| |_  ";
       if (input[i] == '[')
           artwork +="| |   ";
       if (input[i] == '{')
           artwork +=" | |  ";
       if (input[i]== ']')
           artwork +="  | | ";
       if (input[i] == '}')
           artwork +=" | |  ";
       if (input[i] == '|')
           artwork +="| | ";
       if (input[i] == '\\')
           artwork +=" \\ \\    ";
       if (input[i] == ';')
           artwork +="    ";
       if (input[i] == ':')
           artwork +="(_) ";
       if (input[i] == '\'')
           artwork +="|/  ";
       if (input[i] == '"')
           artwork +=" V V  ";
       if (input[i] == '<')
           artwork +=" / /  ";
       if (input[i] == ',')
           artwork +="    ";
       if (input[i] == '>')
           artwork +=" \\ \\  ";
       if (input[i] == '/')
           artwork +="   / /  ";
       if (input[i] == '?')
           artwork +="   ) | ";
   }
   artwork += Leader;
   //loop will print fourth layer
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
           artwork +="|  _  | ";
       if (input[i] == 'B' || input[i] == 'b')
           artwork +="| ___ \\ ";
       if (input[i] == 'C' || input[i] == 'c')
           artwork +="| |     ";
       if (input[i] == 'D' || input[i] == 'd')
           artwork +="| | | | ";
       if (input[i] == 'E' || input[i] == 'e')
           artwork +="|  __|  ";
       if (input[i] == 'F' || input[i] == 'f')
           artwork +="|  _|   ";
       if (input[i] == 'G' || input[i] == 'g')
           artwork +="| | __  ";
       if (input[i] == 'H' || input[i] == 'h')
           artwork +="|  _  | ";
       if (input[i] == 'I' || input[i] == 'i')
           artwork +="  | |   ";
       if (input[i] == 'J' || input[i] == 'j')
           artwork +="    | | ";
       if (input[i] == 'K' || input[i] == 'k')
           artwork +="|    \\  ";
       if (input[i] == 'L' || input[i] == 'l')
           artwork +="| |     ";
       if (input[i] == 'M' || input[i] == 'm')
           artwork +="| |\\/| | ";
       if (input[i] == 'N' || input[i] == 'n')
           artwork +="| . ` | ";
       if (input[i] == 'O' || input[i] == 'o')
           artwork +="| | | | ";
       if (input[i] == 'P' || input[i] == 'p')
           artwork +="|  __/  ";
       if (input[i] == 'Q' || input[i] == 'q')
           artwork +="| | | | ";
       if (input[i] == 'R' || input[i] == 'r')
           artwork +="|    /  ";
       if (input[i] == 'S' || input[i] == 's')
           artwork +=" `--. \\ ";
       if (input[i] == 'T' || input[i] == 't')
           artwork +="  | |   ";
       if (input[i] == 'U' || input[i] == 'u')
           artwork +="| | | | ";
       if (input[i] == 'V' || input[i] == 'v')
           artwork +="| | | | ";
       if (input[i] == 'W' || input[i] == 'w')
           artwork +="| |/\\| | ";
       if (input[i] == 'X' || input[i] == 'x')
           artwork +=" / ^ \\  ";
       if (input[i] == 'Y' || input[i] == 'y')
           artwork +="  \\ /   ";
       if (input[i] == 'Z' || input[i]== 'z')
           artwork +="  / /   ";
       if (input[i] == ' ')
           artwork +="  ";
       if (input[i] == '`')
           artwork +="    ";
       if (input[i] == '~')
           artwork +="|/\\/  ";
       if (input[i] == '1')
           artwork +=" | |  ";
       if (input[i]== '2')
           artwork +="  / /   ";
       if (input[i]== '3')
           artwork +="    \\ \\ ";
       if (input[i] == '4')
           artwork +="/ /_| | ";
       if (input[i] == '5')
           artwork +="    \\ \\ ";
       if (input[i] == '6')
           artwork +="| ___ \\ ";
       if (input[i] == '7')
           artwork +="  / /   ";
       if(input[i] == '.')
           artwork +="    ";
       if (input[i] == '8')
           artwork +=" / _ \\  ";
       if (input[i] == '9')
           artwork +="\\____ | ";
       if (input[i] == '0')
           artwork +="|  /| | ";
       if (input[i] == '!')
           artwork +="| | ";
       if (input[i] == '@')
           artwork +="| | (_| | ";
       if (input[i] == '#')
           artwork +=" _| || |_  ";
       if (input[i] == '$')
           artwork +="\\__ \\ ";
       if (input[i] == '%')
           artwork +="  / /   ";
       if (input[i] == '^')
           artwork +="     ";
       if (input[i] == '&')
           artwork +=" / _ \\/\\ ";
       if (input[i] == '*')
           artwork +="|_     _| ";
       if (input[i] == '(')
           artwork +="| |  ";
       if (input[i] == ')')
           artwork +=" | | ";
       if (input[i] == '-')
           artwork +="|______| ";
       if (input[i] == '_')
           artwork +="         ";
       if (input[i] == '=')
           artwork +=" ______  ";
       if (input[i] == '+')
           artwork +="|_   _| ";
       if (input[i] == '[')
           artwork +="| |   ";
       if (input[i] == '{')
           artwork +="< <   ";
       if (input[i]== ']')
           artwork +="  | | ";
       if (input[i] == '}')
           artwork +="  > > ";
       if (input[i] == '|')
           artwork +="| | ";
       if (input[i] == '\\')
           artwork +="  \\ \\   ";
       if (input[i] == ';')
           artwork +=" _  ";
       if (input[i] == ':')
           artwork +="    ";
       if (input[i] == '\'')
           artwork +="    ";
       if (input[i] == '"')
           artwork +="      ";
       if (input[i] == '<')
           artwork +="< <   ";
       if (input[i] == ',')
           artwork +=" _  ";
       if (input[i] == '>')
           artwork +="  > > ";
       if (input[i] == '/')
           artwork +="  / /   ";
       if (input[i] == '?')
           artwork +="  / /  ";
   }
   artwork += Leader;
   //loop will print fifth layer
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
           artwork +="| | | | ";
       if (input[i] == 'B' || input[i] == 'b')
           artwork +="| |_/ / ";
       if (input[i] == 'C' || input[i] == 'c')
           artwork +="| \\__/\\ ";
       if (input[i] == 'D' || input[i] == 'd')
           artwork +="| |/ /  ";
       if (input[i] == 'E' || input[i] == 'e')
           artwork +="| |___  ";
       if (input[i] == 'F' || input[i] == 'f')
           artwork +="| |     ";
       if (input[i] == 'G' || input[i] == 'g')
           artwork +="| |_\\ \\ ";
       if (input[i] == 'H' || input[i] == 'h')
           artwork +="| | | | ";
       if (input[i] == 'I' || input[i] == 'i')
           artwork +=" _| |_  ";
       if (input[i] == 'J' || input[i] == 'j')
           artwork +="/\\__/ / ";
       if (input[i] == 'K' || input[i] == 'k')
           artwork +="| |\\  \\ ";
       if (input[i] == 'L' || input[i] == 'l')
           artwork +="| |____ ";
       if (input[i] == 'M' || input[i] == 'm')
           artwork +="| |  | | ";
       if (input[i] == 'N' || input[i] == 'n')
           artwork +="| |\\  | ";
       if (input[i] == 'O' || input[i] == 'o')
           artwork +="\\ \\_/ / ";
       if (input[i] == 'P' || input[i] == 'p')
           artwork +="| |     ";
       if (input[i] == 'Q' || input[i] == 'q')
           artwork +="\\ \\/' / ";
       if (input[i] == 'R' || input[i] == 'r')
           artwork +="| |\\ \\  ";
       if (input[i] == 'S' || input[i] == 's')
           artwork +="/\\__/ / ";
       if (input[i] == 'T' || input[i] == 't')
           artwork +="  | |   ";
       if (input[i] == 'U' || input[i] == 'u')
           artwork +="| |_| | ";
       if (input[i] == 'V' || input[i] == 'v')
           artwork +="\\ \\_/ / ";
       if (input[i] == 'W' || input[i] == 'w')
           artwork +="\\  /\\  / ";
       if (input[i] == 'X' || input[i] == 'x')
           artwork +="/ / \\ \\ ";
       if (input[i] == 'Y' || input[i] == 'y')
           artwork +="  | |   ";
       if (input[i] == 'Z' || input[i]== 'z')
           artwork +="./ /___ ";
       if (input[i] == ' ')
           artwork +="  ";
       if (input[i] == '`')
           artwork +="    ";
       if (input[i] == '~')
           artwork +="      ";
       if (input[i] == '1')
           artwork +="_| |_ ";
       if (input[i]== '2')
           artwork +="./ /___ ";
       if (input[i]== '3')
           artwork +=".___/ / ";
       if (input[i] == '4')
           artwork +="\\___  | ";
       if (input[i] == '5')
           artwork +="/\\__/ / ";
       if (input[i] == '6')
           artwork +="| \\_/ | ";
       if (input[i] == '7')
           artwork +="./ /    ";
       if(input[i] == '.')
           artwork +=" _  ";
       if (input[i] == '8')
           artwork +="| |_| | ";
       if (input[i] == '9')
           artwork +=".___/ / ";
       if (input[i] == '0')
           artwork +="\\ |_/ / ";
       if (input[i] == '!')
           artwork +="|_| ";
       if (input[i] == '@')
           artwork +=" \\ \\__,_| ";
       if (input[i] == '#')
           artwork +="|_  __  _| ";
       if (input[i] == '$')
           artwork +="(   / ";
       if (input[i] == '%')
           artwork +=" / / _  ";
       if (input[i] == '^')
           artwork +="     ";
       if (input[i] == '&')
           artwork +="| (_>  < ";
       if (input[i] == '*')
           artwork +=" / , . \\  ";
       if (input[i] == '(')
           artwork +="| |  ";
       if (input[i] == ')')
           artwork +=" | | ";
       if (input[i] == '-')
           artwork +="         ";
       if (input[i] == '_')
           artwork +=" ______  ";
       if (input[i] == '=')
           artwork +="|______| ";
       if (input[i] == '+')
           artwork +="  |_|   ";
       if (input[i] == '[')
           artwork +="| |_  ";
       if (input[i] == '{')
           artwork +=" | |  ";
       if (input[i]== ']')
           artwork +=" _| | ";
       if (input[i] == '}')
           artwork +=" | |  ";
       if (input[i] == '|')
           artwork +="| | ";
       if (input[i] == '\\')
           artwork +="   \\ \\  ";
       if (input[i] == ';')
           artwork +="( ) ";
       if (input[i] == ':')
           artwork +=" _  ";
       if (input[i] == '\'')
           artwork +="    ";
       if (input[i] == '"')
           artwork +="      ";
       if (input[i] == '<')
           artwork +=" \\ \\  ";
       if (input[i] == ',')
           artwork +="( ) ";
       if (input[i] == '>')
           artwork +=" / /  ";
       if (input[i] == '/')
           artwork +=" / /    ";
       if (input[i] == '?')
           artwork +=" |_|   ";
   }
   artwork += Leader;
   //loop will print sixth layer
   for (uint32_t i = 0; i < stringLength; i++)
   {
       if (input[i] == 'A' || input[i]== 'a')
           artwork +="\\_| |_/ ";
       if (input[i] == 'B' || input[i] == 'b')
           artwork +="\\____/  ";
       if (input[i] == 'C' || input[i] == 'c')
           artwork +=" \\____/ ";
       if (input[i] == 'D' || input[i] == 'd')
           artwork +="|___/   ";
       if (input[i] == 'E' || input[i] == 'e')
           artwork +="\\____/  ";
       if (input[i] == 'F' || input[i] == 'f')
           artwork +="\\_|     ";
       if (input[i] == 'G' || input[i] == 'g')
           artwork +=" \\____/ ";
       if (input[i] == 'H' || input[i] == 'h')
           artwork +="\\_| |_/ ";
       if (input[i] == 'I' || input[i] == 'i')
           artwork +=" \\___/  ";
       if (input[i] == 'J' || input[i] == 'j')
           artwork +="\\____/  ";
       if (input[i] == 'K' || input[i] == 'k')
           artwork +="\\_| \\_/ ";
       if (input[i] == 'L' || input[i] == 'l')
           artwork +="\\_____/ ";
       if (input[i] == 'M' || input[i] == 'm')
           artwork +="\\_|  |_/ ";
       if (input[i] == 'N' || input[i] == 'n')
           artwork +="\\_| \\_/ ";
       if (input[i] == 'O' || input[i] == 'o')
           artwork +=" \\___/  ";
       if (input[i] == 'P' || input[i] == 'p')
           artwork +="\\_|     ";
       if (input[i] == 'Q' || input[i] == 'q')
           artwork +=" \\_/\\_\\ ";
       if (input[i] == 'R' || input[i] == 'r')
           artwork +="\\_| \\_| ";
       if (input[i] == 'S' || input[i] == 's')
           artwork +="\\____/  ";
       if (input[i] == 'T' || input[i] == 't')
           artwork +="  \\_/   ";
       if (input[i] == 'U' || input[i] == 'u')
           artwork +=" \\___/  ";
       if (input[i] == 'V' || input[i] == 'v')
           artwork +=" \\___/  ";
       if (input[i] == 'W' || input[i] == 'w')
           artwork +=" \\/  \\/  ";
       if (input[i] == 'X' || input[i] == 'x')
           artwork +="\\/   \\/ ";
       if (input[i] == 'Y'  || input[i] == 'y')
           artwork +="  \\_/   ";
       if (input[i] == 'Z' || input[i]== 'z')
           artwork +="\\_____/ ";
       if (input[i] == ' ')
           artwork +="  ";
       if (input[i] == '`')
           artwork +="    ";
       if (input[i] == '~')
           artwork +="      ";
       if (input[i] == '1')
           artwork +="\\___/ ";
       if (input[i]== '2')
           artwork +="\\_____/ ";
       if (input[i]== '3')
           artwork +="\\____/  ";
       if (input[i] == '4')
           artwork +="    |_/ ";
       if (input[i] == '5')
           artwork +="\\____/  ";
       if (input[i] == '6')
           artwork +="\\_____/ ";
       if (input[i] == '7')
           artwork +="\\_/     ";
       if(input[i] == '.')
           artwork +="(_) ";
       if (input[i] == '8')
           artwork +="\\_____/ ";
       if (input[i] == '9')
           artwork +="\\____/  ";
       if (input[i] == '0')
           artwork +=" \\___/  ";
       if (input[i] == '!')
           artwork +="(_) ";
       if (input[i] == '@')
           artwork +="  \\____/  ";
       if (input[i] == '#')
           artwork +="  |_||_|   ";
       if (input[i] == '$')
           artwork +=" |_|  ";
       if (input[i] == '%')
           artwork +="/_/ (_) ";
       if (input[i] == '^')
           artwork +="     ";
       if (input[i] == '&')
           artwork +=" \\___/\\/ ";
       if (input[i] == '*')
           artwork +=" \\/|_|\\/  ";
       if (input[i] == '(')
           artwork +=" \\_\\ ";
       if (input[i] == ')')
           artwork +="/_/  ";
       if (input[i] == '-')
           artwork +="         ";
       if (input[i] == '_')
           artwork +="|______| ";
       if (input[i] == '=')
           artwork +="         ";
       if (input[i] == '+')
           artwork +="        ";
       if (input[i] == '[')
           artwork +="|___| ";
       if (input[i] == '{')
           artwork +="  \\_\\ ";
       if (input[i]== ']')
           artwork +="|___| ";
       if (input[i] == '}')
           artwork +="/_/   ";
       if (input[i] == '|')
           artwork +="|_| ";
       if (input[i] == '\\')
           artwork +="    \\_\\ ";
       if (input[i] == ';')
           artwork +="|/  ";
       if (input[i] == ':')
           artwork +="(_) ";
       if (input[i] == '\'')
           artwork +="    ";
       if (input[i] == '"')
           artwork +="      ";
       if (input[i] == '<')
           artwork +="  \\_\\ ";
       if (input[i] == ',')
           artwork +="|/  ";
       if (input[i] == '>')
           artwork +="/_/   ";
       if (input[i] == '/')
           artwork +="/_/     ";
       if (input[i] == '?')
           artwork +=" (_)   ";
   }
   return artwork;
}
