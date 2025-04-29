import yaml

def mergeYamlFiles(file: str = None,
                plugin_file: str = None) -> dict:
    '''
    @brief Utility function to merge two ROS 2 YAML parameter files.
    @param file: The base YAML file whit targets.
    @param plugin_file: The plugin YAML file.

    This script defines a function that reads two YAML files—one for the base configuration 
    and one for a plugin—and merges their `ros__parameters`. It also extracts a list of target keys 
    if available and injects them as a separate 'target' list.
    '''
    with open(file, 'r', encoding='utf-8') as f1, open(plugin_file, 'r', encoding='utf-8') as f2:
        file1 = yaml.safe_load(f1) or {}
        file2 = yaml.safe_load(f2) or {}
        file1 = file1[list(file1.keys())[0]]['ros__parameters']
        file2 = file2['plugin']['ros__parameters']

    def recursiveMerge(dict1: dict, dict2: dict) -> dict:
        '''
        @brief Recursive function to merge two dictionaries.
        @param dict1: The first dictionary.
        @param dict2: The second dictionary.
        @return: The merged dictionary.

        '''
        for key, value in dict2.items():
            if key in dict1 and isinstance(dict1[key], dict) and isinstance(value, dict):
                recursiveMerge(dict1[key], value)
            else:
                dict1[key] = value
        return dict1

    parameters = recursiveMerge(file1, file2)

    if ('targets' in file1):
        targets = list(file1.get('targets').keys())
        target = []
        for _ in range (len(targets)):
            target.append(str(targets[_]))
        target_dict = {'target': target}

        parameters.update(target_dict)
    
<<<<<<< HEAD:fbot_world_bringup/scripts/merge_yaml_files.py
    return parameters
=======
    return parameters
>>>>>>> main:scripts/merge_yaml_files.py
