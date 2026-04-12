Scriptname DynamicHDT hidden

;PARAM:
;   new_physics_file_path   should be relative path which should look like the original xml path in nif file.
;   on_ARMA_item     should be referring to an ArmorAddon(ARMA) object for this to function.
;   persist          whether to remember the ArmorAddon object so as to avoid resetting physics file on loading save or re-equipping.
;
;Return:    true upon success
bool Function ReloadPhysicsFile(Actor on_actor, ArmorAddon on_ARMA_item, String new_physics_file_path, Bool persist, Bool verbose_log = false) native global

;PARAM:
;   new_physics_file_path   should be relative path which should look like the original xml path in nif file.
;   old_physics_file_path   original path of the physics file that's been written in the nif file.
;   persist          whether to remember old_physics_file_path.
;
;Return:    true upon success
bool Function SwapPhysicsFile(Actor on_actor, String old_physics_file_path, String new_physics_file_path, Bool persist, Bool verbose_log = false) native global

String Function QueryCurrentPhysicsFile(Actor on_actor, ArmorAddon on_ARMA_item, Bool verbose_log = false) native global

;PARAM:
;   actor       the actor whose physics bones to toggle.
;   boneNames   array of bone name strings (e.g. "NPC L Breast", "HDT Belly").
;   on          true = enable physics (dynamic), false = disable physics (kinematic/frozen).
;
;Return:    Bool array of same length as boneNames. Each entry is the previous state of that bone
;           (true = was dynamic, false = was kinematic or not found).
Bool[] Function TogglePhysics(Actor actor, String[] boneNames, Bool on) native global

;PARAM:
;   actor   the actor to reset.
;   full    true = full reset: bones snap to reference pose, all velocity lost.
;           false = soft reset: physics systems rebuilt but current poses and velocities are kept.
;
Function ResetPhysics(Actor actor, Bool full) native global
