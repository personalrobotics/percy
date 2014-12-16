import pickle

f = open('tag_matrices.pkl')
tag_dict = pickle.load(f)
pros_tag_list = {}
for tag in tag_dict['prosilica_tags']:
    pros_tag_list[tag] = []
    for mat in tag_dict['prosilica_tags'][tag]:
        pros_tag_list[tag].append(mat.tolist())

xtion_tag_list = {}

for tag in tag_dict['xtion_tags']:
    xtion_tag_list[tag] = []
    for mat in tag_dict['xtion_tags'][tag]:
        xtion_tag_list[tag].append(mat.tolist())

tag_dict_list = {'pros_tags':pros_tag_list,'xtion_tags':xtion_tag_list}
f2 = open('tag_lists.pkl','w')
pickle.dump(tag_dict_list,f2)
f2.close()
