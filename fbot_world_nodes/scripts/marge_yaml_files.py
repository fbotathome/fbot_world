import yaml

def MergeYamlFiles(file, plugin, output_file, node_name=None):
    with open(file, 'r', encoding='utf-8') as f1, open(plugin, 'r', encoding='utf-8') as f2:
        data1 = yaml.safe_load(f1) or {}
        data2 = yaml.safe_load(f2) or {}
        data_aux = data1
        data1 = data1[list(data1.keys())[0]]
        data2 = data2['plugin']

        print (data1.get('ros__parameters').get('targets').keys())
    def recursive_merge(dict1, dict2):
        for key, value in dict2.items():
            if key in dict1 and isinstance(dict1[key], dict) and isinstance(value, dict):
                recursive_merge(dict1[key], value)
            else:
                dict1[key] = value
        return dict1

    final = recursive_merge(data1, data2)

    if ('targets' in data1.get('ros__parameters')):
        lugares = list(data1.get('ros__parameters').get('targets').keys())
        lugares_string = []
        for x in range (len(lugares)):
            lugares_string.append(str(lugares[x]))
        lugares1 =[]
        lugares1.append(lugares)
        lugares1 = {'lugares': lugares1}

        final['ros__parameters'].update(lugares1)


    data = {
        node_name: final
    }

    #merged_data = {list(data_aux.keys())[0]: (final, 'lugares: {'+ str(lugares)+'}')}
    
    with open(output_file, 'w', encoding='utf-8') as f_out:
        valor = yaml.dump(data, f_out, allow_unicode=True, default_flow_style=False, sort_keys=False)