/**
 * @file ConsolePort.h
 * @brief ConsolePort class declaration
 *
 * @section intro_sec Introduction
  *  Contains the declaration of the "singleton-like" ConsolePort class. ConsolePort employ lazy initialization,
  *   so it is initialized the first time it is used.  The behavior utilizes interrupt handling to interact
  *   with the peripheral.
 *
 * @note This class is thread and interrupt-safe.
 */

#ifndef ConsolePort_h
#define ConsolePort_h

#include <atomic>
#include <cstdio>
#include <cstring>

#include <driver/uart.h>
#include "UTILITIES/common/Utility.h"
#include "UTILITIES/common/Mutex.h"
#include "UTILITIES/common/CommonIDs.h"

//=============================================================================//
/// MACROS
//=============================================================================//

    //===================================//
    ///   CONSOLE GUI RELATED CONSTANTS
    //===================================//

/**
 * @brief Maximum float data fields required in frame ID messages.
 */
#define MAX_FLOAT_DATA_FIELDS   20

/**
 * @brief Byte size of a float on the current platform. Typically 4 bytes.
 */
#define FLOAT_SIZE_IN_BYTES     4

/**
 * @brief Data size, excluding the initial ID and operation bytes.
 */
#define DATA_SIZE               (MAX_FLOAT_DATA_FIELDS * FLOAT_SIZE_IN_BYTES)

/**
 * @brief Header size, for including a byte of the message ID and another for the OPERATION
 */
#define HEADER_SIZE             2

/**
 * @brief Total byte size of the data segment in a message.
 */
#define FRAME_DATA_SIZE        (HEADER_SIZE + DATA_SIZE)

/**
 * @brief Byte size of the CRC (CRC16).
 */
#define FRAME_CRC_SIZE          2

/**
 * @brief Maximum message size between GUI and Controller, including CRC.
 */
#define CONTROL_MESSAGE_FRAME_SIZE  (FRAME_DATA_SIZE + FRAME_CRC_SIZE)

/**
 * @brief Position of the ID in the frame.
 */
#define FRAME_ID_POS            0

/**
 * @brief Position of the operation code in the frame.
 */
#define FRAME_OPERATION_POS     1

/**
 * @brief Starting position of data bytes in the frame.
 */
#define FRAME_FIRST_DATA_POS    2

/**
 * @brief Position of the CRC in the frame, after the data segment.
 */
#define FRAME_CRC_POS           (FRAME_DATA_SIZE)

/**
 * @brief Max number of bytes used in a Console GUI to Controller message to speed up processing.
 */
#define CONSOLE_TO_CONTROLLER_MESSAGES_DATA_FIELDS_USED     36

/**
 * @brief Indicates component actuation failure in status messages.
 */
#define COMPONENT_FAILURE   0x00

/**
 * @brief Indicates component actuation success in status messages.
 */
#define COMPONENT_SUCCESS   0x01

    //=============================================//
    ///           COMPILE-TIME VALIDATIONS
    //=============================================//

#if CONSOLE_TO_CONTROLLER_MESSAGES_DATA_FIELDS_USED > DATA_SIZE
#error "CONSOLE_TO_CONTROLLER_MESSAGES_DATA_FIELDS_USED should not be greater than DATA_SIZE!"
#endif

#if (MAX_FLOAT_DATA_FIELDS * 4) > DATA_SIZE
#error "Maximum possible float data exceeds DATA_SIZE!"
#endif

#if FRAME_DATA_SIZE > CONTROL_MESSAGE_FRAME_SIZE
#error "FRAME_DATA_SIZE cannot be greater than CONTROL_MESSAGE_FRAME_SIZE!"
#endif

#if (FRAME_DATA_SIZE + FRAME_CRC_SIZE) != CONTROL_MESSAGE_FRAME_SIZE
#error "The sum of FRAME_DATA_SIZE and FRAME_CRC_SIZE should equal CONTROL_MESSAGE_FRAME_SIZE!"
#endif

#if FRAME_FIRST_DATA_POS >= DATA_SIZE
#error "FRAME_FIRST_DATA_POS position should be less than DATA_SIZE!"
#endif

#if FRAME_CRC_POS != (CONTROL_MESSAGE_FRAME_SIZE - FRAME_CRC_SIZE)
#error "FRAME_CRC_POS should be at the position (CONTROL_MESSAGE_FRAME_SIZE - FRAME_CRC_SIZE)!"
#endif

    //===================================//
    ///   CONSOLE WRITE RELATED MACROS
    //===================================//

/**
 * @brief Indicates that a timestamp should be prepended to the message in WriteWithNoNewline.
 */
#define ADD_TIMESTAMP   true

/**
 * @brief Indicates that NO timestamp should be added to the message in WriteWithNoNewline.
 */
#define NO_TIMESTAMP    false

    //===================================//
    ///   CONSOLE WRITE RELATED MACROS
    //===================================//

/**
 * @brief Conditionally writes a formatted log message.
 *
 * This macro conditionally calls the `ConsolePort::Write` function based on the value of the `verboseFlag`.
 * If `verboseFlag` is true, the formatted message is written; otherwise, the call is optimized out.
 *
 * The `do { ... } while (0)` construct ensures that the macro behaves like a single statement, even if it
 * contains multiple statements. This helps avoid potential issues with macro expansion in different contexts,
 * such as within `if` statements.
 *
 * @param verboseFlag A boolean flag that determines whether the log message should be written.
 * @param format A printf-style format string.
 * @param ... Additional arguments for the format string.
 *
 * @note The `do { ... } while (0)` construct ensures proper scoping and single-statement behavior.
 *
 * @code
 * bool verbose = true;
 * WRITE_CONDITIONAL(verbose, "This is a conditional log message: %d\n", 42);
 * @endcode
 */
#define WRITE_CONDITIONAL(verboseFlag, format, ...) \
    do { \
        if (verboseFlag) { \
            ConsolePort::Write(format, ##__VA_ARGS__); \
        } \
    } while (0)

/**
 * @brief Conditionally writes a formatted log message at the specified log level.
 *
 * This macro conditionally calls the `ConsolePort::WriteLog` function based on the value of the `verboseFlag`.
 * If `verboseFlag` is true and the specified log level is greater than or equal to the current log level,
 * the formatted message is written.
 *
 * The `do { ... } while (0)` construct ensures that the macro behaves like a single statement, even if it
 * contains multiple statements. This helps avoid potential issues with macro expansion in different contexts,
 * such as within `if` statements.
 *
 * @param verboseFlag A boolean flag that determines whether the log message should be written.
 * @param level The log level at which the message should be written.
 * @param format A printf-style format string.
 * @param ... Additional arguments for the format string.
 *
 * @note The `do { ... } while (0)` construct ensures proper scoping and single-statement behavior.
 *
 * @code
 * bool verbose = true;
 * WRITE_LOG_CONDITIONAL(verbose, LogLevel::DEBUG, "This is a conditional log message: %d\n", 42);
 * @endcode
 */
#define WRITE_LOG_CONDITIONAL(verboseFlag, level, format, ...) \
    do { \
        if (verboseFlag && (level >= ConsolePort::GetInstance().GetLogLevel())) { \
            ConsolePort::WriteLog(level, format, ##__VA_ARGS__); \
        } \
    } while (0)


//=============================================================================//
/// FUNCTIONS
//=============================================================================//
/// ------------------------------------ Declarations of 'C-callable' functions --------------------------------------------------
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief C-callable variable argument list debug diagnostic output function that time-stamps data and writes it to the serial port.
 *
 * @param format A printf-style format string.
 * @param ... Variadic parameters based on the format string.
 */
void ConsoleWrite(const char* format, ...);

#ifdef __cplusplus
}

/// ------------------------------------ End of 'C-callable' functions --------------------------------------------------

#include <string>
#include "UTILITIES/common/CircularBuffer.h"

extern "C" void ConsoleWrite(const char* format, ...);

/// Used for receiving from & sending command data to the Console app
struct commandFrame
{
    uint8_t component;
    uint8_t operation;
    uint8_t data[DATA_SIZE];
};

/// Used to send sensor data to the Console app.
struct sensorFrame
{
    uint8_t component;
    uint8_t operation;
    float data[MAX_FLOAT_DATA_FIELDS];
};

class ConsolePort
{
public:

	//========================================================//
	/// INSTANCE GETTER
	//========================================================//

	/**
	 * @brief Get the singleton instance of ConsolePort
	 *
	 * @return ConsolePort& Singleton instance of ConsolePort
	 */
	static ConsolePort& GetInstance() noexcept;

	/**
	 * @brief Ensures class is properly initialized.
	 * @return true if initialized, false otherwise.
	 */
	bool EnsureInitialized() noexcept
	{
		if (!initialized)
		{
			initialized = Initialize();
		}
		return initialized;
	}

	//========================================================//
	/// PROJECT ASCII ART WRITER
	//========================================================//

	/**
	 * @brief Creates an ascii art from string that can be printed.
	 * @param input Character string to be formatted into the ASCII art.
	 * @return String representation of the ASCII art.
	 */
	static std::string CreateArtFromString( const char* input ) noexcept;

	/**
	 * @brief Prints Projects Ascii Art
	 */
	static void PrintProjectAsciiArt() noexcept;

	/**
	 * @brief Writes the string as art.
	 * @param message string to convert to ascii art.
	 */
	static void WriteAsciiArt(const char* message) noexcept;

	//========================================================//
	/// HELPERS
	//========================================================//

	/**
	 * @brief Get the ANSI escape code for the specified color.
	 *
	 * @param color The color code enum value.
	 * @return const char* The ANSI escape code string for the specified color.
	 */
	static const char* GetColorCode(ColorCode color) noexcept;

	/**
	 * @brief Get the ANSI escape code for the specified text formatting.
	 *
	 * @param format The formatting code enum value.
	 * @return const char* The ANSI escape code string for the specified formatting.
	 */
	static const char* GetFormatCode(FormatCode format) noexcept;

	/**
	 * @brief Get the combined ANSI escape codes for the specified text formatting.
	 *
	 * @param formats A vector of formatting code enum values.
	 * @return std::string The combined ANSI escape code string for the specified formatting.
	 */
	static std::string GetFormatCode(const std::vector<FormatCode>& formats) noexcept;

	/**
	 * @brief Retrieves the color code for the specified log level.
	 *
	 * @param level The log level (DEBUG, INFO, WARN, ERROR).
	 * @return The ANSI color code corresponding to the log level.
	 */
	static const char* GetLogLevelColor(LogLevel level);

	//========================================================//
	/// SEMIHOST TERMINAL WRITERS
	//========================================================//

	/**
	 * @brief Variable argument list debug diagnostic output function that time-stamps data and writes it to the serial port.
	 *
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteSemihost(const char* format, ...) noexcept;

	//========================================================//
	/// SERIAL CONSOLE WRITERS
	//========================================================//

	//==================================//
	/// ENABLERS/DISABLERS
	//==================================//

    /**
     * @brief Check if output of debug data over the serial port is enabled.
     * @return True if output is enabled, false otherwise.
     */
	bool IsOutputEnabled() noexcept { return outputEnabled; }

	/**
	 * @brief Enable output of debug data over the serial port.
	 */
	bool EnableOutput() noexcept;

	/**
	 * @brief Disable output of debug data over the serial port.
	 */
	bool DisableOutput() noexcept;


    /**
     * @brief Check if input of debug data over the serial port is enabled.
     * @return True if input is enabled, false otherwise.
     */
	bool IsInputEnabled() noexcept { return inputEnabled; }

	/**
	 * @brief Enable input of debug data over the serial port.
	 */
	bool EnableInput() noexcept;

	/**
	 * @brief Disable input of debug data over the serial port.
	 */
	bool DisableInput() noexcept;

	//==================================//
	/// RECEIVED DATA CHECKERS/READERS
	//==================================//

	/**
	 * @brief Checks if there's data available in the receive buffer.
	 */
	bool IsDataAvailable() noexcept;

	/**
	 * @brief Reads and return from the FIFO receive buffer
	 * @param data  Reference to where the received data will be stored
	 * @return  True if read was successful, false otherwise
	 */
	bool Read(uint8_t& data) noexcept;

	/**
	 * @brief Waits for a character receive event.
	 *
	 * @param waitOption ThreadX ticks wait option, [default TX_WAIT_FOREVER]
	 *
	 * @return Return True if wait was successful, false otherwise
	 */
	bool WaitForCharReceiveEvent(ULONG waitOption = TX_WAIT_FOREVER) noexcept;

	/**
	 * @brief Waits for a transmit buffer emptied event.
	 *
	 * @param waitOption ThreadX ticks wait option, [default TX_WAIT_FOREVER]
	 *
	 * @return Return True if wait was successful, false otherwise
	 */
	bool WaitForTransmitBufferEmptiedEvent(ULONG waitOption = TX_WAIT_FOREVER) noexcept;

	//================================//
	/// LOGGING LEVEL BASED WRITERS
	//================================//

	/**
	 * @brief Sets the current log level.
	 *
	 * @param level The log level to set (DEBUG, INFO, WARN, ERROR).
	 */
	static void SetLogLevel(LogLevel level);

	/**
	 * @brief Gets the current log level
	 * @return current log level.
	 */
	static LogLevel GetLogLevel();


	/**
	 * @brief Logs messages based on the log level and color-codes them.
	 *
	 * @param level The log level of the message.
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteLog(LogLevel level, const char* format, ...) noexcept;

	/**
	 * @brief Logs messages conditionally based on the log level and verbosity flag.
	 *
	 * @param verboseFlag Local "verbosity" flag to control the amount of diagnostic output.
	 * @param level The log level of the message.
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteLogConditional(bool verboseFlag, LogLevel level, const char* format, ...) noexcept;

	//================================//
	/// STRAIGHT TO CONSOLE WRITERS
	//================================//

	/**
	 * @brief Writes messages with default behavior (with timestamp, newline).
	 *
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void Write(const char* format, ...) noexcept;

	/**
	 * @brief Writes messages conditionally based on the verbosity flag.
	 *
	 * @param verboseFlag Local "verbosity" flag to control the amount of diagnostic output.
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteConditional(bool verboseFlag, const char* format, ...) noexcept;


	/**
	 * @brief Writes a colored message to the console port.
	 *
	 * This function formats a message with the specified ANSI color code and writes it to the console.
	 *
	 * @param format A printf-style format string.
	 * @param colorCode The color code to use.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteColored(const char* format, ColorCode colorCode, ...) noexcept;

	/**
	 * @brief Writes a formatted message to the console port.
	 *
	 * This function formats a message with the specified ANSI formatting code and writes it to the console.
	 *
	 * @param format A printf-style format string.
	 * @param formatCode The formatting code to use.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteFormatted(const char* format, FormatCode formatCode, ...) noexcept;

	/**
	 * @brief Writes a colored and formatted message to the console port.
	 *
	 * This function formats a message with the specified ANSI color and formatting codes and writes it to the console.
	 *
	 * @param format A printf-style format string.
	 * @param colorCode The color code to use.
	 * @param formatCode The formatting code to use.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteColoredFormatted(const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept;



	/**
	 * @brief Writes messages without a timestamp.
	 *
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteNoTimestamp(const char* format, ...) noexcept;

	/**
	 * @brief Writes a colored line without a timestamp.
	 * @param format The format string.
	 * @param colorCode The color code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteColoredNoTimestamp(const char* format, ColorCode colorCode, ...) noexcept;

	/**
	 * @brief Writes a formatted line without a timestamp.
	 * @param format The format string.
	 * @param formatCode The format code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteFormattedNoTimestamp(const char* format, FormatCode formatCode, ...) noexcept;

	/**
	 * @brief Writes a colored and formatted line without a timestamp.
	 * @param format The format string.
	 * @param colorCode The color code to use.
	 * @param formatCode The format code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteColoredFormattedNoTimestamp(const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept;



	/**
	 * @brief Writes messages without newline handling (optionally with a timestamp).
	 *
	 * @param addTimestamp True if a timestamp is wanted, false if not.
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteWithNoNewline(bool addTimestamp, const char* format, ...) noexcept;

	/**
	 * @brief Writes a colored line without newline handling.
	 * @param addTimestamp Whether to add a timestamp.
	 * @param format The format string.
	 * @param colorCode The color code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteColoredWithNoNewline(bool addTimestamp, const char* format, ColorCode colorCode, ...) noexcept;

	/**
	 * @brief Writes a formatted line without newline handling.
	 * @param addTimestamp Whether to add a timestamp.
	 * @param format The format string.
	 * @param formatCode The format code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteFormattedWithNoNewline(bool addTimestamp, const char* format, FormatCode formatCode, ...) noexcept;

	/**
	 * @brief Writes a colored and formatted line without newline handling.
	 * @param addTimestamp Whether to add a timestamp.
	 * @param format The format string.
	 * @param colorCode The color code to use.
	 * @param formatCode The format code to use.
	 * @param ... Additional arguments for the format string.
	 */
	static void WriteColoredFormattedWithNoNewline(bool addTimestamp, const char* format, ColorCode colorCode, FormatCode formatCode, ...) noexcept;



	/**
	 * @brief Writes messages throttled by a minimum interval.
	 *
	 * @param format A printf-style format string.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteThrottled(const char* format, ...) noexcept;


	/**
	 * @brief Writes messages with execution time profiling.
	 *
	 * @param format A printf-style format string.
	 * @param startTime The start time for profiling.
	 * @param ... Variadic parameters based on the format string.
	 */
	static void WriteWithTiming(uint32_t startTime, const char* format, ...) noexcept;

	/**
	 * @brief Logs JSON-formatted messages.
	 *
	 * @param key The key for the JSON message.
	 * @param value The value for the JSON message.
	 */
	static void WriteJSON(const char* key, const char* value) noexcept;


	/**
	 * @brief Just writes the carriage return and new line characters.
	 */
	static void NewLine() noexcept;

	//====================================//
	/// SERIAL CONSOLE DATA BLOCK WRITERS
	//====================================//

	/**
	 * @brief This function outputs a block of binary data in hex format, extending over several lines if necessary.
	 *
	 * The function takes a printf-style format string, a block of binary data, and the size of the data in bytes.
	 * It formats the data as a sequence of hex bytes and outputs it to the console port. The data will be split
	 * across multiple lines if necessary.
	 *
	 * @param format A printf-style format string.
	 * @param data A pointer to the data to be output as a sequence of hex bytes.
	 * @param dataSizeBytes The number of bytes in the data block.
	 * @param ... Variable argument list to match the format string.
	 *
	 * @example
	 * // Example usage:
	 * uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
	 * ConsolePort::WriteDataBlock("Data Block:", data, sizeof(data));
	 *
	 * // This will output:
	 * // #D 0.00.00.00.000: Data Block:
	 * // #D (8 bytes): 0xDE 0xAD 0xBE 0xEF 0x01 0x02 0x03 0x04
	 */
	static void WriteDataBlock(  const char* format, const void* data, uint32_t dataSizeBytes,  ... ) noexcept;

	//=====================================//
	/// SERIAL CONSOLE NOTIFICATION WRITERS
	//=====================================//

	/**
	 * @brief This function prints a formatted string to the console port.
	 *
	 * The function takes an input string and two integers specifying the number of spaces and asterisks for padding.
	 * It then generates a formatted string, with the input string surrounded by the specified number of spaces and asterisks.
	 * The formatted string is printed to the console port, sandwiched between two lines of '=' characters.
	 * The length of the '=' character lines is calculated based on the length of the formatted string.
	 *
	 * @param input_string The string to be formatted and printed to the console port.
	 * @param num_spaces The number of spaces for padding on each side of the input string.
	 * @param num_stars The number of asterisks for padding on each side of the input string.
	 *
	 * @example
	 * /// Example usage:
	 * 	- printFormattedString("SI MANIFOLD CONTROL LOOP HAS ENCOUNTERED AN ERROR", 4, 4);
	 *
	 * /// This will output:
	 * /// ===========================================================================
	 * /// ===||****    SI MANIFOLD CONTROL LOOP HAS ENCOUNTERED AN ERROR    ****||===
	 * /// ===========================================================================
	 */
	static void WriteToNotify(const char* input_string, int num_spaces = 4, int num_stars = 4) noexcept;

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
	static void WriteToNotifyCustom( const std::string& message,
			int paddingLength = 10,
			char paddingChar = '*',
			char borderChar = '=',
			char sideBorderChar = '|',
			int borderLength = 75,
			bool verboseInit = true ) noexcept;


	/**
	 * @brief This function prints formatted strings to the console port.
	 *
	 * The function takes multiple input strings and two integers specifying the number of spaces and asterisks for padding.
	 * It then generates formatted strings, with each input string surrounded by the specified number of spaces and asterisks.
	 * The formatted strings are printed to the console port, sandwiched between two lines of '=' characters.
	 * The length of the '=' character lines is calculated based on the length of the longest formatted string.
	 *
	 * @param input_strings A vector of strings to be formatted and printed to the console port.
	 * @param num_spaces The number of spaces for padding on each side of the input strings.
	 * @param num_stars The number of asterisks for padding on each side of the input strings.
	 *
	 * @example
	 * /// Example usage:
	 * std::vector<std::string> messages = {
	 *     "SI MANIFOLD CONTROL LOOP HAS ENCOUNTERED AN ERROR",
	 *     "SYSTEM INITIALIZING",
	 *     "UPDATE COMPLETE"
	 * };
	 * ConsolePort::WriteToNotify(messages, 4, 4);
	 *
	 * /// This will output:
	 * /// ===========================================================================
	 * /// ===||****    SI MANIFOLD CONTROL LOOP HAS ENCOUNTERED AN ERROR    ****||===
	 * /// ===||****    SYSTEM INITIALIZING    ****||===
	 * /// ===||****    UPDATE COMPLETE    ****||===
	 * /// ===========================================================================
	 */
	static void WriteToNotify(const std::vector<std::string>& input_strings, int num_spaces = 4, int num_stars = 4) noexcept;

	/**
	 * @brief Displays formatted messages on the console with customizable padding, border, and side characters.
	 *
	 * This function allows you to create attention-grabbing messages on the terminal with various formatting options.
	 * The messages are padded with customizable characters, enclosed within borders, and centered in the display area.
	 *
	 * @param messages A vector of strings representing the main content to be displayed.
	 * @param paddingLength The number of padding characters to use on either side of the messages. Default is 10.
	 * @param paddingChar The character used to pad around the messages. Default is '*'.
	 * @param borderChar The character used for the top and bottom borders. Default is '='.
	 * @param sideBorderChar The character used for the side borders. Default is '|'.
	 * @param borderLength The total length of the border line. Default is 75.
	 * @param verboseInit A boolean flag to determine if the messages should be displayed. Default is true.
	 *
	 * @example
	 * // Example usage:
	 * std::vector<std::string> messages = {
	 *     "PRESS POWER BUTTON",
	 *     "SYSTEM INITIALIZING",
	 *     "UPDATE COMPLETE"
	 * };
	 * ConsolePort::WriteToNotifyCustom(messages, 5, '*', '=', '|', 75, true);
	 *
	 * // Example output:
	 * //
	 * // =======================================================================
	 * // |||*****                  PRESS POWER BUTTON                   *****|||
	 * // |||*****                  SYSTEM INITIALIZING                  *****|||
	 * // |||*****                  UPDATE COMPLETE                      *****|||
	 * // =======================================================================
	 */
	static void WriteToNotifyCustom(const std::vector<std::string>& messages,
	                                int paddingLength = 10,
	                                char paddingChar = '*',
	                                char borderChar = '=',
	                                char sideBorderChar = '|',
	                                int borderLength = 75,
	                                bool verboseInit = true) noexcept;

	//========================================================//
	/// SPECIALIZED SERIAL CONSOLE WRITERS TO CONSOLE APP
	//========================================================//

	/**
	 * @brief Sends the byte array to the serial port the console GUI is connected to.
	 *
	 * @param send_buffer A pointer to the buffer containing the data to send.
	 * @param size The size of the data in the buffer.
	 * @param log_message A boolean flag to tell console GUI to log or not.
	 *                    Appends 0x01 for logging and 0x00 for non-logging after #C.
	 */
	static void WriteConsoleControl(uint8_t *send_buffer, size_t size, bool log_message = true) noexcept;

	/**
	 * @brief Sends a command frame to the application through the console port.
	 *
	 * This method prepares the frame, calculates its CRC and sends it to the application
	 * using the UART port.
	 *
	 * @param frame The command frame containing component, operation, and data.
	 *
	 * @note The method is marked as noexcept, implying it does not throw exceptions.
	 *
	 * @see crc16
	 * @see WriteConsoleControl
	 */
	void SendToApp(commandFrame frame) noexcept;

	/**
	 * @brief Sends a sensor frame to the application through the console port.
	 *
	 * This method prepares the frame, calculates its CRC, and sends it to the application
	 * using the UART port.
	 *
	 * @param frame The sensor frame containing component, operation, and data.
	 *
	 * @note The method is marked as noexcept, implying it does not throw exceptions.
	 *
	 * @see crc16
	 * @see WriteConsoleControl
	 */
	void SendToApp(sensorFrame frame) noexcept;

	//========================================================//
	//========================================================//

private:

	//========================================================//
	/// CLASS CONSTRUCTORS/INITIALIZERS
	//========================================================//

	/**
	 * @brief Constructor for ConsolePort class
        * @param txUart UART port to initialize ConsolePort with.
	 * @note This constructor is marked explicit to prevent automatic conversion.
	 */
        explicit ConsolePort(uart_port_t txUart) noexcept;

	/**
	 * @brief Copy constructor.
	 * @details This constructor is deleted to prevent copying of instances.
	 * @param The other instance of ConsolePort to be copied (not used).
	 */
	ConsolePort(const ConsolePort&) = delete;

	/**
	 * @brief Assignment operator.
	 * @details This operator is deleted to prevent copying of instances.
	 * @param The other instance of ConsolePort to be copied (not used).
	 * @return A reference to this instance.
	 */
	ConsolePort& operator =(const ConsolePort&) = delete;

	/**
	 * @brief Destructor for ConsolePort class
	 * @details Destructor is simple, it disables interrupts and closes the port.
	 */
	virtual ~ConsolePort() noexcept;

	/**
	 * @brief Initializes the singleton class.
	 * @return true if initialization successful, false otherwise.
	 */
	virtual bool Initialize() noexcept;

	//========================================================//
	/// INTERRUPT HANDLERS
	//========================================================//

        /**
         * @brief Handles UART events (placeholder for ESP-IDF callbacks).
         */
        void HandleInterrupt(const uart_event_t* event) noexcept;

	//========================================================//
	/// BUFFER APPENDERS
	//========================================================//

	/**
	 * @brief Append a set of values to the txbuffer.
	 *
	 * @param data Address of the data.
	 * @param sizeBytes Number of elements.
	 *
	 * @return true On successful append operation.
	 */
	bool WriteAppend(const char* data, uint16_t sizeBytes) noexcept;

	/**
	 * @brief Append a set of values to the rxbuffer.
	 *
	 * @param data Address of the data.
	 * @param sizeBytes Number of elements.
	 *
	 * @return true On successful append operation.
	 */
	bool ReadAppend(const char* data, uint16_t sizeBytes) noexcept;

	//========================================================//
	/// HELPERS
	//========================================================//

	/**
	 * @brief Writes a formatted string to the console port.
	 *
	 * This function formats a message and writes it to the console port with optional color and formatting.
	 *
	 * @param format A printf-style format string.
	 * @param args Variadic parameters based on the format string.
	 * @param colorCode The ANSI color code to use (optional).
	 * @param formatCode The ANSI formatting code to use (optional).
	 */
	static void WriteSmallBuffer(const char* format, va_list args, const char* colorCode = nullptr, const char* formatCode = nullptr) noexcept;

	/**
	 * @brief Writes colored messages using ANSI color codes for terminals.
	 *
	 * @param format A printf-style format string.
	 * @param colorCode The ANSI color code to use.
	 * @param args Variadic parameters based on the format string.
	 */
	static void WriteColoredSmallBuffer(const char* format, const char* colorCode, va_list args) noexcept;

	/**
	 * @brief Writes formatted messages using ANSI formatting codes for terminals.
	 *
	 * This function formats a message with the specified ANSI formatting code and writes it to the console.
	 *
	 * @param format A printf-style format string.
	 * @param formatCode The ANSI formatting code to use.
	 * @param args Variadic parameters based on the format string.
	 */
	static void WriteFormattedSmallBuffer(const char* format, const char* formatCode, va_list args) noexcept;

	/**
	 * @brief Writes colored and formatted messages using ANSI color and formatting codes for terminals.
	 *
	 * This function formats a message with the specified ANSI color and formatting codes and writes it to the console.
	 *
	 * @param format A printf-style format string.
	 * @param colorCode The ANSI color code to use.
	 * @param formatCode The ANSI formatting code to use.
	 * @param args Variadic parameters based on the format string.
	 */
	static void WriteColoredFormattedSmallBuffer(const char* format, const char* colorCode, const char* formatCode, va_list args) noexcept;

	void WriteNotifySmallBuffer(const char* format, va_list args) noexcept;

	/**
	 * @brief Writes a formatted string or message to the console port with additional options.
	 *
	 * This function takes a format string or message, a variable argument list, and additional options
	 * for timestamping, newline addition, truncation, semihosting, color, and formatting. It formats the
	 * string or message and writes it to the console port.
	 *
	 * @tparam BufferSize The size of the buffer used for formatting.
	 * @param addTimestamp Whether to add a timestamp to the formatted string or message.
	 * @param addNewline Whether to add a newline character to the formatted string or message.
	 * @param input The format string or message.
	 * @param args The variable argument list for the format string (ignored if isFormat is false).
	 * @param isFormat Whether the input is a format string (true) or a message (false).
	 * @param allowTruncate Whether to allow truncation of the formatted string or message if it exceeds the buffer size.
	 * @param writeSemihost Whether to use semihosting for writing the formatted string or message.
	 * @param colorCode The ANSI escape code for the text color (optional).
	 * @param formatCode The ANSI escape code for the text formatting (optional).
	 */
	template <size_t BufferSize>
	static void WriteImpl(bool addTimestamp, bool addNewline, const char* input, va_list args, bool isFormat, bool allowTruncate, bool writeSemihost, const char* colorCode = nullptr, const char* formatCode = nullptr) noexcept;

	/**
	 * @brief Helper function to write a formatted string or message to the console port.
	 *
	 * This function performs the actual writing logic, including formatting the string or message,
	 * adding timestamps, handling newlines, managing truncation, semihosting, color, and formatting.
	 *
	 * @param buffer The buffer used for formatting.
	 * @param bufferSize The size of the buffer.
	 * @param addTimestamp Whether to add a timestamp to the formatted string or message.
	 * @param addNewline Whether to add a newline character to the formatted string or message.
	 * @param input The format string or message.
	 * @param args The variable argument list for the format string (ignored if isFormat is false).
	 * @param isFormat Whether the input is a format string (true) or a message (false).
	 * @param allowTruncate Whether to allow truncation of the formatted string or message if it exceeds the buffer size.
	 * @param writeSemihost Whether to use semihosting for writing the formatted string or message.
	 * @param colorCode The ANSI escape code for the text color (optional).
	 * @param formatCode The ANSI escape code for the text formatting (optional).
	 */
	static void WriteImplHelper(char* buffer, size_t bufferSize, bool addTimestamp, bool addNewline, const char* input, va_list args, bool isFormat, bool allowTruncate, bool writeSemihost, const char* colorCode = nullptr, const char* formatCode = nullptr) noexcept;

	//========================================================//
	/// KEY VARIABLES
	//========================================================//

        uart_port_t txUart;

	std::atomic<bool> inputEnabled;
	std::atomic<bool> outputEnabled;
	bool initialized;

	volatile uint8_t readData;

	volatile uint8_t receivedChar[1]; /**< Static place that will store next received char. */

	Mutex txMutex;	/// Mutex to lock down the transmitting through the control interface port
    static const char txMutexName[];

	Mutex rxMutex;	/// Mutex to lock down the receiving through the control interface port
	static const char rxMutexName[];

	bool txMutexExternalLock;
	bool rxMutexExternalLock;

    LogLevel currentLogLevel;  /// Default log level
    uint32_t lastWriteTime;  /// Us6ed for write throttling

	//========================================================//
	/// WRITE AND READ DATA BUFFERS
	//========================================================//

	static constexpr uint16_t CircularBufferSizeBytes = 10000;
	static constexpr uint16_t FormatSmallBufferSizeBytes = 250;
	static constexpr uint16_t FormatLargeBufferSizeBytes = 500;

	CircularBuffer<uint8_t, CircularBufferSizeBytes> txBuffer;
	CircularBuffer<uint8_t, CircularBufferSizeBytes> rxBuffer;

	//========================================================//
	/// CONSOLE EVENT FLAGS
	//========================================================//

	/// Add an event flag group to signal the command event
	TX_EVENT_FLAGS_GROUP consoleEventFlags;
	static const char eventFlagsName[];
	bool consoleEventFlagsCreated;
    bool isTransmitBufferEmptiedFlagSet;

	ULONG consoleReceiveEventFlagPos = 0x01; /// Example: Check for flag 0x01
	ULONG consoleTransmitBufferEmptiedFlagPos = 0x02; /// Example: Check for flag 0x02

	//========================================================//
	/// CALLBACKS AND FRIENDS
	//========================================================//

	friend void ConsoleWrite(const char* format, ...);
};

template <size_t BufferSize>
void ConsolePort::WriteImpl(bool addTimestamp, bool addNewline, const char* input, va_list args, bool isFormat, bool allowTruncate, bool writeSemihost, const char* colorCode, const char* formatCode) noexcept {
    if (!GetInstance().outputEnabled || !GetInstance().EnsureInitialized()) {
        return;
    }

    char buffer[BufferSize];
    WriteImplHelper(buffer, BufferSize, addTimestamp, addNewline, input, args, isFormat, allowTruncate, writeSemihost, colorCode, formatCode);
}

#endif /// Called from c++ code

#endif /// Class
