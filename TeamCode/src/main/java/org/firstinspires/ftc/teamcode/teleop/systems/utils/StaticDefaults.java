package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.OnCreate;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import dalvik.system.DexFile;

/**
 * Remembers the starting value of every static field in TeamCode and can put them all back (ResetStaticValues).
 *
 * Static fields keep their value between opmode runs until the Robot Controller app restarts, so a change from
 * Panels, a tuner or a previous opmode (transportState, trims, goal side, the auto's end pose, ...) carries over.
 * Java can't re-run a class's initializers, so instead this records every value once, when the app starts
 * (@OnCreate, before anything has had a chance to change them), and resetAll() copies them back.
 *
 * What gets reset, for every class in org.firstinspires.ftc.teamcode:
 *  - every static field that isn't final
 *  - the contents of arrays and of TeamCode objects held in static fields (e.g. CRShooter.values, a ShooterConfig),
 *    so Panels edits inside them are undone too, and the same object is kept
 * Objects from other libraries (Pedro configs, Poses, hardware) only get their reference put back, not their contents.
 */
public class StaticDefaults {
    private static final String TAG = "StaticDefaults";
    private static final String TEAMCODE_PACKAGE = "org.firstinspires.ftc.teamcode.";
    private static final int MAX_DEPTH = 4; // how deep to follow TeamCode objects inside TeamCode objects

    /** The recorded starting value of one field (or one object's contents). */
    private static class Saved {
        Object ref;                  // the original value (or object)
        Object arrayCopy;            // the original contents, if ref is an array
        Map<Field, Saved> fields;    // the original contents, if ref is a TeamCode object
    }

    private static final Map<Field, Saved> savedStatics = new LinkedHashMap<>();
    private static boolean snapshotTaken = false;
    private static int classesSkipped = 0;

    /** Called by the SDK when the Robot Controller app starts. */
    @OnCreate
    public static void onAppStart(Context context) {
        snapshot(context);
    }

    public static boolean hasSnapshot() {
        return snapshotTaken;
    }

    public static int fieldCount() {
        return savedStatics.size();
    }

    /** Classes that couldn't be loaded when the snapshot was taken (their fields aren't reset). */
    public static int classesSkipped() {
        return classesSkipped;
    }

    /** Records every TeamCode static field. Only runs once per app start. */
    private static synchronized void snapshot(Context context) {
        if (snapshotTaken) return;

        List<String> classNames = new ArrayList<>();
        try {
            DexFile dex = new DexFile(context.getPackageCodePath());
            for (Enumeration<String> e = dex.entries(); e.hasMoreElements(); ) {
                String name = e.nextElement();
                if (name.startsWith(TEAMCODE_PACKAGE)) classNames.add(name);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "couldn't list the TeamCode classes, nothing will be reset");
            return;
        }

        ClassLoader loader = StaticDefaults.class.getClassLoader();
        for (String name : classNames) {
            try {
                // true = run the class's static initializers now, so the recorded values are the ones in the code
                Class<?> c = Class.forName(name, true, loader);
                if (c == StaticDefaults.class || c.isEnum() || c.isInterface() || c.isAnnotation()) continue;
                for (Field f : c.getDeclaredFields()) {
                    if (!Modifier.isStatic(f.getModifiers()) || f.isSynthetic()) continue;
                    // A final primitive or String can never change, nothing to record
                    if (Modifier.isFinal(f.getModifiers()) && (f.getType().isPrimitive() || f.getType() == String.class)) continue;
                    f.setAccessible(true);
                    savedStatics.put(f, capture(f.get(null), 0));
                }
            } catch (Throwable t) {
                classesSkipped++;
                RobotLog.ww(TAG, "skipped " + name + ": " + t);
            }
        }

        snapshotTaken = true;
        RobotLog.ii(TAG, "recorded " + savedStatics.size() + " static fields, skipped " + classesSkipped + " classes");
    }

    /**
     * Puts every recorded static field back to its starting value.
     * @return the fields that had changed, as "Class.field" (sorted), empty if nothing changed
     */
    public static synchronized List<String> resetAll() {
        List<String> changed = new ArrayList<>();
        if (!snapshotTaken) return changed;

        for (Map.Entry<Field, Saved> entry : savedStatics.entrySet()) {
            Field f = entry.getKey();
            Saved saved = entry.getValue();
            try {
                if (differs(saved, f.get(null))) {
                    changed.add(f.getDeclaringClass().getSimpleName() + "." + f.getName());
                }
                Object original = restore(saved);
                // A final field's reference can't change; restore() already put its contents back
                if (!Modifier.isFinal(f.getModifiers())) f.set(null, original);
            } catch (Throwable t) {
                RobotLog.ww(TAG, "couldn't reset " + f + ": " + t);
            }
        }

        Collections.sort(changed);
        return changed;
    }

    // -------------------------------- RECORD / RESTORE ---------------------------------

    private static Saved capture(Object value, int depth) throws IllegalAccessException {
        Saved s = new Saved();
        s.ref = value;
        if (value == null) return s;

        if (value.getClass().isArray()) {
            s.arrayCopy = copyArray(value);
        } else if (isTeamCodeObject(value) && depth < MAX_DEPTH) {
            s.fields = new LinkedHashMap<>();
            for (Field f : instanceFields(value.getClass())) {
                s.fields.put(f, capture(f.get(value), depth + 1));
            }
        }
        return s;
    }

    /** Puts the recorded contents back into the original object and returns it. */
    private static Object restore(Saved s) throws IllegalAccessException {
        if (s.ref == null) return null;

        if (s.arrayCopy != null) {
            // Copy back into the original array: code may have changed it in place (Panels) or swapped it (LutTuner)
            System.arraycopy(s.arrayCopy, 0, s.ref, 0, Array.getLength(s.arrayCopy));
        } else if (s.fields != null) {
            for (Map.Entry<Field, Saved> entry : s.fields.entrySet()) {
                Field f = entry.getKey();
                Object original = restore(entry.getValue());
                if (!Modifier.isFinal(f.getModifiers())) f.set(s.ref, original);
            }
        }
        return s.ref;
    }

    /** True if current is different from the recorded value (compares contents for arrays and TeamCode objects). */
    private static boolean differs(Saved s, Object current) throws IllegalAccessException {
        if (s.arrayCopy != null) {
            return current == null || current.getClass() != s.arrayCopy.getClass()
                    || !Objects.deepEquals(s.arrayCopy, current);
        }
        if (s.fields != null) {
            if (current != s.ref) return true;
            for (Map.Entry<Field, Saved> entry : s.fields.entrySet()) {
                if (differs(entry.getValue(), entry.getKey().get(current))) return true;
            }
            return false;
        }
        return !Objects.equals(s.ref, current);
    }

    private static Object copyArray(Object array) {
        int length = Array.getLength(array);
        Object copy = Array.newInstance(array.getClass().getComponentType(), length);
        System.arraycopy(array, 0, copy, 0, length);
        return copy;
    }

    private static boolean isTeamCodeObject(Object value) {
        Class<?> c = value.getClass();
        return c.getName().startsWith(TEAMCODE_PACKAGE) && !c.isEnum() && !c.isSynthetic();
    }

    /** The non-static fields of c and of its TeamCode superclasses. */
    private static List<Field> instanceFields(Class<?> c) {
        List<Field> fields = new ArrayList<>();
        for (Class<?> k = c; k != null && k.getName().startsWith(TEAMCODE_PACKAGE); k = k.getSuperclass()) {
            for (Field f : k.getDeclaredFields()) {
                if (Modifier.isStatic(f.getModifiers()) || f.isSynthetic()) continue;
                f.setAccessible(true);
                fields.add(f);
            }
        }
        return fields;
    }
}
