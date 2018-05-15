# -*- coding: utf-8 -*-
import json



# # Define data
# data = {'a list': [1, 42, 3.141, 1337, 'help', u'€'],
#         'a string': 'bla',
#         'another dict': {'foo': 'bar',
#                          'key': 'value',
#                          'the answer': 42}}
#
# # Write JSON file
# with open('data.json', 'w', encoding='utf8') as outfile:
#     str_ = json.dumps(data,
#                       indent=4, sort_keys=True,
#                       separators=(',', ': '), ensure_ascii=False)
#     outfile.write(str_)

# Read JSON file
with open('map.json') as data_file:
    data_loaded = json.load(data_file)

# print(data == data_loaded)
print(data_loaded)