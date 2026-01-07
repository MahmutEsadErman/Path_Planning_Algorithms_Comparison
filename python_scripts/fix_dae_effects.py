
import xml.etree.ElementTree as ET
import sys

input_file = '/ros2_tutorials/new_models/nb_park/meshes/nb_park.dae'
output_file = input_file

# Register namespace to avoid ns0 prefixes
ET.register_namespace('', "http://www.collada.org/2005/11/COLLADASchema")

try:
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    
    effects = root.findall('.//c:library_effects/c:effect', ns)
    
    count = 0
    for effect in effects:
        profile = effect.find('c:profile_COMMON', ns)
        if profile is None:
            continue
            
        # Find all texture nodes
        textures = profile.findall('.//c:texture', ns)
        
        # Track which images we've already created params for in this effect
        processed_images = set()
        
        # We need to insert newparams before the technique
        # So we collect them and insert them at index 0
        new_params = []
        
        for tex in textures:
            image_id = tex.get('texture')
            
            # Skip if it already looks like a sampler (e.g. ends in -sampler)
            if not image_id or image_id.endswith('-sampler'):
                continue
                
            sampler_sid = image_id + "-sampler"
            surface_sid = image_id + "-surface"
            
            # Point texture to the sampler
            tex.set('texture', sampler_sid)
            
            if image_id not in processed_images:
                # Create Surface Newparam
                np_surf = ET.Element('{http://www.collada.org/2005/11/COLLADASchema}newparam', sid=surface_sid)
                surf = ET.SubElement(np_surf, '{http://www.collada.org/2005/11/COLLADASchema}surface', type="2D")
                init = ET.SubElement(surf, '{http://www.collada.org/2005/11/COLLADASchema}init_from')
                init.text = image_id
                
                # Create Sampler Newparam
                np_samp = ET.Element('{http://www.collada.org/2005/11/COLLADASchema}newparam', sid=sampler_sid)
                samp = ET.SubElement(np_samp, '{http://www.collada.org/2005/11/COLLADASchema}sampler2D')
                src = ET.SubElement(samp, '{http://www.collada.org/2005/11/COLLADASchema}source')
                src.text = surface_sid
                
                new_params.append(np_surf)
                new_params.append(np_samp)
                
                processed_images.add(image_id)
                count += 1

        # Insert new params at the beginning of profile_COMMON
        # Reverse to maintain order if we insert at 0 repeatedly
        for np in reversed(new_params):
            profile.insert(0, np)
            
    tree.write(output_file, encoding='UTF-8', xml_declaration=True)
    print(f"Updated {count} texture references.")

except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
