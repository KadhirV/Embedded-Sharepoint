import os
from datetime import datetime
from pathlib import Path
directory =  os.getcwd() # Obtains current working directory

class colors:
    black = "\u001b[30m"
    red = "\u001b[31m"
    green = "\u001b[32m"
    yellow = "\u001b[33m"
    blue = "\u001b[34m"
    magenta = "\u001b[35m"
    cyan = "\u001b[36m"
    white = "\u001b[37m"
    reset = "\u001b[0m"

copyright = 'Copyright (c) 2018-' + str(datetime.now().year) + ' UT Longhorn Racing Solar'

for root, subdirectories, files in os.walk(directory):
    for file in files:
        filepath = os.path.join(root, file)
        if (file.endswith(".c") or file.endswith('.h')) and \
            ("STM32F4" not in filepath) and ("CMSIS" not in filepath) and ("RTOS" not in filepath):
            # finds all .c .h .py files
            # ignores files which are not ours
            print(colors.yellow + filepath + colors.reset)

            # User prompt
            operation = input(colors.yellow + "Change this file? (y/n/q)> " + colors.reset)
            if operation == 'q': quit()
            if operation != 'y': continue

            # Open file
            f = open(filepath,"r",encoding = "ISO-8859-1")  #meant to fix bug with not recognizing byte 0x92
            text = f.readlines()

            # Look for existing comment
            firstline = text[0]
            end = -1
            existing_brief = None

            if firstline[:2] == '/*':
                # find where multi-line comment ends
                end = 0
                while end < len(text) and '*/' not in text[end]:
                    end += 1
                ml_comment = text[:end+1]

                # check if brief exists
                brief_ind = 0

                while brief_ind < len(ml_comment) and '@brief' not in ml_comment[brief_ind]:
                    brief_ind += 1
                
                if brief_ind == len(ml_comment):
                    print(colors.red + "No brief found!" + colors.reset)
                else:
                    brief_end = brief_ind+1
                    while brief_end < len(ml_comment) and '@' not in ml_comment[brief_end] and '*/' not in ml_comment[brief_end] and '***' not in ml_comment[brief_end]:
                        brief_end += 1
                    existing_brief = ml_comment[brief_ind:brief_end-1]
                    print(colors.green + "Existing brief found, \n" + colors.blue + '\n'.join(existing_brief) + colors.reset)

            elif firstline[:2] == '//':
                end = 0

            # Prompt for existing brief
            keep = False
            if existing_brief is not None:
                keep = (input(colors.yellow + "Keep existing brief? (y/n)> " + colors.reset) != 'n')

            if keep:
                brief = existing_brief
            else:
                brief = [' * @brief \n']
            
            # Construct comment
            comment = []
            comment.append('/**\n')
            comment.append(' * @copyright ' + copyright + '\n')
            comment.append(' * @file ' + file + '\n')
            comment.extend(brief)
            comment.append(' * \n')

            if file.endswith('.h'): comment.append(' * @defgroup ' + file[:-2] + '\n')
            
            comment.append(' * @addtogroup ' + file[:-2] + '\n')
            comment.append(' * @{\n')
            comment.append(' */\n')

            # Insert comment
            text[:end+1] = comment
            
            # Add end of group comment to end of file
            endgroup = len(text) - 1
            while endgroup >= 0 and text[endgroup] == "":
                endgroup -= 1
            
            if '/*@}*/' in text[endgroup]: text[endgroup] = '\n/* @} */\n'
            elif text[endgroup] != '/* @} */': text.append('\n/* @} */\n')

            if endgroup+2 < len(text): del text[endgroup+2:]

            # Print comment and last few lines
            print(colors.blue + '\n'.join(comment) + '\n\n    ...\n\n    ...\n\n    ...\n\n' + '\n'.join(text[-2:]) + colors.reset)

            # Reconfirm
            if input(colors.yellow + 'Write to file? (y/n)> ' + colors.reset) == 'y':
                out = open(filepath, 'w')
                out.writelines(text)
                out.close()
                print(colors.green + 'Wrote to file!' + colors.reset)