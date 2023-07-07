import os
import re
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
visited = set() # Will only look at each file once (not twice for header and src)

def run():
    for root, subdirectories, files in os.walk(directory, topdown=True):
        subdirectories.sort(key=lambda x : 1 if 'Inc' in x else 0)  # Go to src before includes
        
        for file in files:
            filepath = os.path.join(root, file)
            if (file.endswith(".c") or file.endswith('.h')) and \
                ("StdPeriph" not in filepath) and ("HAL" not in filepath) and ("CMSIS" not in filepath) and ("RTOS" not in filepath):
                
                if file[:-2] in visited: continue

                print(filepath)

                # User prompt
                operation = input("Update file comment? (y/n/q)> ")
                if operation == 'q': quit()
                if operation == 'y':
                    update_file_comment(filepath)
                
                visited.add(file[:-2])

                
def update_file_comment(filepath):
    # Get text from file
    text = get_text(filepath)

    # Find src comment
    file_comment, start, end = find_file_comment(text)

    header_comment = []
    header_path = ''

    # If src file, find corresponding header comment
    if filepath.endswith(".c"):
        # Look for header file
        header_path = find_header_file(filepath)
        header_text = get_text(header_path)
        header_comment, start_h, end_h = find_header_comment(header_text)
    
    # Print src comment and header comment side by side
    print_side_by_side(file_comment, header_comment, filepath, header_path)

    # Prompt for picking brief
    brief = pick_brief(file_comment, header_comment)
    if brief[0] == 'skip':
        print(colors.green + 'Skipping both files!' + colors.reset)
        return

    # Generate new file comment(s)
    print(colors.green + 'Generating new file comments!' + colors.reset)
    new_file_comment = generate_file_comment(filepath, brief)
    new_hdr_comment = []
    if header_path: new_hdr_comment = generate_file_comment(header_path, brief)
    
    # Print and prompt
    print_side_by_side(new_file_comment, new_hdr_comment, filepath, header_path)
    
    confirm_changes = input("Confirm changes to these files? (y/n)> ")
    
    if confirm_changes != 'y': 
        print(colors.red + "Changes rejected! Continuing..." + colors.reset)
        return
    
    # Update text contents
    print(colors.green + "Changes accepted! Writing to files..." + colors.reset)
    
    text[start:end] = new_file_comment

    set_text(filepath, text)

    if new_hdr_comment: 
        header_text[start_h:end_h] = new_hdr_comment
        set_text(header_path, header_text)

def find_start_of_comment(text):
    start = 0
    while start < len(text) and \
        ('//' not in text[start] and '/*' not in text[start] and '#include' not in text[start]):
        start += 1
    if start == len(text): return -1
    if '#include' in text[start]: return -1
    return start

def find_end_of_comment(text, start):
    end = start
    if text[start][:2] == '//': # Single line comment
        return end
    elif text[start][:2] == '/*':
        while end < len(text) and '*/' not in text[end]:
            end += 1
        return end
    return -1

def find_file_comment_pos(text):
    # Look for existing file comment in file
    start = find_start_of_comment(text)
    if start == -1: 
        print(colors.red + "No file comment found in current file!" + colors.reset)
        return (0, 0)
    else:
        end = find_end_of_comment(text, start)
        if end == -1:
            print(colors.red + "Malformed comment!" + colors.reset)
    
    return (start, end+1)

def find_file_comment(text):
    # Look for existing file comment in file
    start, end = find_file_comment_pos(text)
    return text[start:end], start, end

def find_header_file(filepath):
    # Find header file based on src
    if not filepath.endswith('.c'):
        print(colors.red + "Invalid input!" + colors.reset)
        return ''

    inc_dir = os.path.dirname(os.path.dirname(filepath)) 
    if 'BSP' in inc_dir:
        inc_dir = os.path.dirname(inc_dir)
    
    inc_dir += '/Inc/'
    file_name = os.path.basename(filepath)[:-2] + ".h"
    for root, dir, files in os.walk(inc_dir):
        if file_name in files:
            header_path = os.path.join(inc_dir, file_name)
            print(colors.green + "Header found!" + colors.reset)
            return header_path
    
    return ''

def find_header_comment(header_text):
    if header_text == '':
        return [],-1,-1
    
    # Find file comment
    start_h, end_h = find_file_comment_pos(header_text)

    header_comment = header_text[start_h:end_h]

    return header_comment, start_h, end_h

def get_text(filepath):
    if filepath == '':
        return ''
    f = open(filepath,"r",encoding = "ISO-8859-1")  #meant to fix bug with not recognizing byte 0x92
    text = f.readlines()
    f.close()
    return text

def set_text(filepath, text):
    if filepath == '':
        return
    f = open(filepath,"w",encoding = "ISO-8859-1")  #meant to fix bug with not recognizing byte 0x92
    f.write("".join(text))
    f.close()

def find_brief(comment):
    brief = []
    found = False
    
    for line in comment:
        if '@brief' in line:
            found = True
            brief.append(line)
            continue
        
        if found == True:
            if '@' in line or '*/' in line: return brief
            brief.append(line)
    return brief

def generate_file_comment(filepath, brief):
    comment = []
    if filepath == '': return comment
    
    basename = os.path.basename(filepath)
    wout_extension = basename[:-2]

    comment.append('/**\n')
    comment.append(' * @copyright ' + copyright + '\n')
    comment.append(' * @file ' + basename + '\n')
    if brief == []: comment.append(' * @brief \n')
    else: comment.extend(brief)
    
    if filepath.endswith('.h'): comment.append(' * @defgroup ' + wout_extension + '\n')
    comment.append(' * @addtogroup ' + wout_extension + '\n')
    comment.append(' * @{\n')
    comment.append('*/\n')
    return comment

def pick_brief(file_comment, header_comment):
    # Prompt for picking brief
    brief_choice = ''
    file_brief = find_brief(file_comment)
    header_brief = find_brief(header_comment)

    if file_brief and header_brief: # Both exist
        while brief_choice.casefold() != 'left' and \
            brief_choice.casefold() != 'right' and \
            brief_choice.casefold() != 'new' and \
            brief_choice.casefold() != 'skip':
            brief_choice = input("Which brief would you like to pick? (left/right/new/skip)> ")
    elif file_brief or header_brief: # Only one exists
        while brief_choice.casefold() != 'keep' and \
            brief_choice.casefold() != 'new' and \
            brief_choice.casefold() != 'skip':
            brief_choice = input("Would you like to keep the existing brief? (keep/new/skip)> ")
        
        if brief_choice.casefold() == 'keep': 
            if file_brief: brief_choice = 'left'
            elif header_brief: brief_choice = 'right'
    else:
        brief_choice = 'new'

    brief = []
    if brief_choice == 'left':
        brief = find_brief(file_comment)
    elif brief_choice == 'right':
        brief = find_brief(header_comment)
    elif brief_choice == 'new':
        brief = []
    elif brief_choice == 'skip':
        brief = ['skip']
    
    return brief

def print_side_by_side(a, b, filepath_a, filepath_b):
    maxlen = len(max(a, key=len)) if a else 0
    for i in range(max(len(a), len(b))):
        a_line = a[i] if i < len(a) else ''
        b_line = b[i] if i < len(b) else '\n'
        print((colors.blue + "{:<"+str(maxlen)+"}").format(a_line).replace('\n','') + '\t' + b_line + colors.reset)

    # Print paths
    print((colors.yellow + "{:<"+str(maxlen)+"}").format(filepath_a).replace('\n','') + '\t' + filepath_b + colors.reset)


run()