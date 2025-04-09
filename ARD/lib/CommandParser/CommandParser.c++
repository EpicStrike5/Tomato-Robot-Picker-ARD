#include "CommandParser.h"

CommandParser::CommandParser() : inputBuffer(""), commandReady(false), commandName(""), argCount(0) {}

// Reads serial until newline '\n' is received
bool CommandParser::readSerial()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();
        if (receivedChar == '\n' || receivedChar == '\r')
        { // Command terminator
            if (inputBuffer.length() > 0)
            {
                parseCommand(inputBuffer);
                inputBuffer = ""; // Clear buffer for next command
                return true;      // Indicate a command was processed
            }
        }
        else
        {
            inputBuffer += receivedChar;
        }
        // Optional: Add a buffer overflow check here
    }
    return false; // No complete command received yet
}

void CommandParser::parseCommand(String commandString)
{
    commandString.trim(); // Remove leading/trailing whitespace
    clearCommand();       // Clear previous command data

    int colonIndex = commandString.indexOf(':');
    if (colonIndex == -1)
    {
        // Invalid format, treat the whole string as command name with 0 args
        commandName = commandString;
        argCount = 0;
        commandReady = true;
        return;
    }

    commandName = commandString.substring(0, colonIndex);
    commandName.trim();

    String argsString = commandString.substring(colonIndex + 1);
    argsString.trim();

    if (argsString.length() > 0)
    {
        int startIndex = 0;
        int commaIndex = -1;
        argCount = 0; // Reset arg count for this parse

        do
        {
            commaIndex = argsString.indexOf(',', startIndex);
            String arg;
            if (commaIndex == -1)
            {
                // Last argument (or only argument)
                arg = argsString.substring(startIndex);
            }
            else
            {
                arg = argsString.substring(startIndex, commaIndex);
            }
            arg.trim();

            if (argCount < MAX_ARGS)
            {
                args[argCount++] = arg;
            }
            else
            {
                // Too many arguments, log error or ignore extras
                Serial.println("Warning: Too many arguments received!");
                break;
            }

            startIndex = commaIndex + 1;

        } while (commaIndex != -1 && argCount < MAX_ARGS);
    }
    else
    {
        argCount = 0; // No arguments after the colon
    }

    commandReady = true;
}

bool CommandParser::isNewCommandAvailable()
{
    return commandReady;
}

String CommandParser::getCommandName()
{
    return commandName;
}

int CommandParser::getNumArgs()
{
    return argCount;
}

float CommandParser::getArgAsFloat(int index)
{
    if (index >= 0 && index < argCount)
    {
        return args[index].toFloat();
    }
    return 0.0f; // Return default value on error
}

long CommandParser::getArgAsLong(int index)
{
    if (index >= 0 && index < argCount)
    {
        return args[index].toInt(); // Using toInt() for long conversion in Arduino context
    }
    return 0L; // Return default value on error
}

String CommandParser::getArgAsString(int index)
{
    if (index >= 0 && index < argCount)
    {
        return args[index];
    }
    return ""; // Return empty string on error
}

void CommandParser::clearCommand()
{
    commandName = "";
    for (int i = 0; i < MAX_ARGS; ++i)
    {
        args[i] = "";
    }
    argCount = 0;
    commandReady = false;
}