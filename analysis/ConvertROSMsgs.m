temp = FileStruct.E1_SUBX0_1C_Left.StimCommand;
temp.Data = temp.Data.Var1;
u = table;

for i=1:length(temp.Data)
    u(i,1:2) = table(struct(temp.Data(i)),temp.Time(i));
      u(i,1) = rmfield(u,{'MD5Checksum','JavaMessage','StdMsgsHeaderClass','Cache','PropertyList','ROSPropertyList'});
end

u.Properties.DimensionNames{1} = 'Data';
u.Properties.DimensionNames{2} = 'Time';
