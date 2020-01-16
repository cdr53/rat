function anim_color = rgb2anim(rgb)
    hexcolor =reshape(sprintf('%02X',rgb.'),6,[]).';
    hexcolor = ['FF',hexcolor];
    bitstring = char(hexToBinaryVector(hexcolor,32)+'0');
    anim_color = typecast(uint32(bin2dec(bitstring)),'int32');
end