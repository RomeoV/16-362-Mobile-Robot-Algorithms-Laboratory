function ca = get_ca_val(c_array, i, n)
    %Gets circular array value of i.
    ca = c_array(mod(i+1,n));
end