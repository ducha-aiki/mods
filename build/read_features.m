fid = fopen('k1.txt');
ndets = fgets(fid);
ndets = str2num(ndets);
det_names = {};
descs_per_det = {};
REGIONS = struct;
for i=1:ndets
    tline = fgets(fid);
    d1 =  strsplit(tline,' ');
    curr_det_name = d1{1};
    det_names{end+1} = curr_det_name;
    curr_num_descs = str2num(d1{2});
    descs_per_det{end+1} = curr_num_descs;
    REGIONS.(curr_det_name) = struct;
    for j=1:curr_num_descs
         tline = fgets(fid);
         d1 =  strsplit(tline,' ');
         curr_desc_name = d1{1};
         curr_desc_num = str2num(d1{2});
         curr_desc_dim = str2num(fgets(fid));
         REGIONS.(curr_det_name).(curr_desc_name) = {};
         for desc_idx=1:curr_desc_num
            tline = fgets(fid);
            tline2 = strrep(tline,'  ',' ');
            raw_region =  strsplit(tline2,' ');
            current_feature = struct;
            current_feature.id = str2num(raw_region{1});
            current_feature.img_id = str2num(raw_region{2});
            current_feature.img_reproj_id = str2num(raw_region{3});
            current_feature.parent_id = str2num(raw_region{4});
            % detected region shape (on synthetised image)
            current_feature.det_x = str2num(raw_region{5});
            current_feature.det_y = str2num(raw_region{6});
            current_feature.det_a11 = str2num(raw_region{7});
            current_feature.det_a12 = str2num(raw_region{8});
            current_feature.det_a21 = str2num(raw_region{9});
            current_feature.det_a22 = str2num(raw_region{10});
            current_feature.det_pyr_scale = str2num(raw_region{11});
            current_feature.det_octave_idx = str2num(raw_region{12});
            current_feature.det_s = str2num(raw_region{13});
            current_feature.det_type = str2num(raw_region{14});
            % reprojected region shape (original image)
            current_feature.x = str2num(raw_region{15});
            current_feature.y = str2num(raw_region{16});
            current_feature.a11 = str2num(raw_region{17});
            current_feature.a12 = str2num(raw_region{18});
            current_feature.a21 = str2num(raw_region{19});
            current_feature.a22 = str2num(raw_region{20});
            current_feature.pyr_scale = str2num(raw_region{21});
            current_feature.octave_idx = str2num(raw_region{22});
            current_feature.s = str2num(raw_region{23});
            current_feature.type = str2num(raw_region{24});
            current_feature.curr_desc_dim = str2num(raw_region{25});
            % descriptor
            curr_desc = [];
            if curr_desc_dim > 0
                curr_desc = zeros(curr_desc_dim,1);
                for dd=1:curr_desc_dim
                    curr_desc(dd,1) = str2num(raw_region{25 + dd});
                end
            end
            current_feature.desc = curr_desc;
             REGIONS.(curr_det_name).(curr_desc_name){end+1} = current_feature;
             
         end
         
    end
end
fclose(fid);
