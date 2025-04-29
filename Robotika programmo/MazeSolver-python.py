def replace_rur_with_s(elements):
    
    i = 0
    j = 0
    result = []
    results = []
    
    while i < len(elements):
        if i + 2 < len(elements) and elements[i] == 'R' and elements[i+1] == 'U' and elements[i+2] == 'R':
            result.append('S')
            i += 3 
        else:
            result.append(elements[i])
            i += 1
    
    print("Output first:", result)
    
    while j < len(result):
        if j + 2 < len(result) and result[j] == 'S' and result[j+1] == 'U' and result[j+2] == 'R':
            results.append('L')
            j += 3  
        elif j + 2 < len(result) and result[j] == 'R' and result[j+1] == 'U' and result[j+2] == 'S':
            results.append('L')
            j += 3  
        else:
            results.append(result[j])
            j += 1
            
    return results

# Contoh penggunaan
input_array = ['R', 'U', 'R', 'R', 'U', 'R', 'U', 'R', 'R', 'U', 'S']
output_array = replace_rur_with_s(input_array)

print("Input :", input_array)
print("Output real:", output_array)