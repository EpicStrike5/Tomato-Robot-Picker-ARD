#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include <Arduino.h>

#define MAX_ARGS 5 // Maximum number of arguments per command

class CommandParser
{
public:
    CommandParser();
    bool readSerial(); // Reads serial input, returns true if a complete command is received
    bool isNewCommandAvailable();
    String getCommandName();
    int getNumArgs();
    float getArgAsFloat(int index);
    long getArgAsLong(int index); // Use long for potential large step counts
    String getArgAsString(int index);
    void clearCommand(); // Clears the current command data

private:
    String inputBuffer;
    bool commandReady;
    String commandName;
    String args[MAX_ARGS];
    int argCount;

    void parseCommand(String commandString);
};

#endif // COMMANDPARSER_H