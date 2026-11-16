"""Resolve old cross-type component-name collisions, without geometry edits."""
def unique_visual_names(model):
    aliases=[]
    for link in model.findall('link'):
        used={c.get('name') for c in link if c.tag!='visual' and c.get('name')}
        for visual in link.findall('visual'):
            old=visual.get('name');new=old
            if new in used:
                new=old+'_visual';suffix=1
                while new in used:new=old+'_visual_'+str(suffix);suffix+=1
                visual.set('name',new)
                aliases.append(dict(link=link.get('name'),source_name=old,imported_name=new))
            used.add(new)
    return aliases
