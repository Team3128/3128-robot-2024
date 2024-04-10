package processor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.TypeElement;

import com.squareup.javapoet.*;
import com.google.auto.service.AutoService;
import javax.tools.Diagnostic;
import javax.lang.model.element.Element;
import javax.lang.model.element.Modifier;
import javax.annotation.processing.Processor;

import common.utility.annotations.*;


@AutoService(Processor.class)
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class AnnotationProcessor extends AbstractProcessor{

    HashMap<String, List<Element>> annotationMap = new HashMap<String, List<Element>>();

    static List<String> generatedClassList = new LinkedList<String>();

    private boolean hasRun = false;

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        //makes sure that the processor is only run once
        if(hasRun){
            return false;
        }

        try{

            annotations.forEach(annotation -> {

                annotationMap.put(annotation.toString(), new ArrayList<Element>());

                roundEnv.getElementsAnnotatedWith(annotation).forEach(
                    element -> {
                        annotationMap.get(annotation.toString()).add(element);
                    }
                );

            });

            processNARUpdateable(annotationMap.get("common.utility.annotations.NARUpdateable"));

            processAutoCommand(annotationMap.get("common.utility.annotations.AutoCommand"));

            processShuffleboardData(annotationMap.get("common.utility.annotations.ShuffleboardData"));
            
            generateClassManger();

            hasRun = true;

        } catch(Exception e){
            e.printStackTrace();
            return false;
        }
        
        return true;
    }

    private boolean handleAnnotationUsage(List<Element> elements, String annotationName){
        boolean isUsed = elements != null && elements.size() > 0;
        if(isUsed){
            print("Generating " + annotationName + " Processor...");
            return true;
        }else{
            print("No " + annotationName + " Annotations...");
            return false;
        }
    }

    public void processShuffleboardData(List<Element> shuffleboardData){
        if(!handleAnnotationUsage(shuffleboardData, "ShuffleboardData")){
            return;
        }

        CodeBlock.Builder codeBlock = CodeBlock.builder();

        for(Element element : shuffleboardData){
            String methodName = element.getSimpleName().toString();
            String tab = element.getAnnotation(ShuffleboardData.class).tab();
            String name = element.getAnnotation(ShuffleboardData.class).name();
            int x = element.getAnnotation(ShuffleboardData.class).x();
            int y = element.getAnnotation(ShuffleboardData.class).y();
            String className = element.getEnclosingElement().getSimpleName().toString();
            String packageName = processingEnv.getElementUtils().getPackageOf(element.getEnclosingElement()).getQualifiedName().toString();
            boolean hasGetInstance = element.getEnclosingElement().getEnclosedElements().stream().anyMatch(e -> e.getSimpleName().toString().equals("getInstance"));
            
            if (hasGetInstance) {
                codeBlock.addStatement("$T.addData($S, $S, ()-> $T.getInstance().$L(), $L, $L)", 
                    ClassName.get("common.utility.shuffleboard", "NAR_Shuffleboard"), 
                    tab, 
                    name, 
                    ClassName.get(packageName, className), 
                    methodName, 
                    x, 
                    y);
            }else{
                codeBlock.addStatement("$T.addData($S, $S, ()-> $T.$L(), $L, $L)", 
                    ClassName.get("common.utility.shuffleboard", "NAR_Shuffleboard"), 
                    tab, 
                    name, 
                    ClassName.get(packageName, className), 
                    methodName, 
                    x, 
                    y);
            }
    

        }

        generateClass("ShuffleboardDataProcessor", codeBlock.build());
    }

    public void processAutoCommand(List<Element> autoCommands){
        if(!handleAnnotationUsage(autoCommands, "AutoCommand")){
            return;
        }

        CodeBlock.Builder codeBlock = CodeBlock.builder();

        for(Element element : autoCommands){
            String methodName = element.getSimpleName().toString();
            String identifier = element.getAnnotation(AutoCommand.class).identifier().toString();
            String className = element.getEnclosingElement().getSimpleName().toString();
            String packageName = processingEnv.getElementUtils().getPackageOf(element.getEnclosingElement()).getQualifiedName().toString();
            boolean hasGetInstance = element.getEnclosingElement().getEnclosedElements().stream().anyMatch(e -> e.getSimpleName().toString().equals("getInstance"));
            
            if(hasGetInstance){
                codeBlock.addStatement("$T.registerCommand($S, $T.getInstance().$L())", 
                    ClassName.get("com.pathplanner.lib.auto", "NamedCommands"), 
                    identifier, 
                    ClassName.get(packageName, className), 
                    methodName);
            }else{
                codeBlock.addStatement("$T.registerCommand($L, $L.$L())", 
                    ClassName.get("com.pathplanner.lib.auto", "NamedCommands"), 
                    identifier, 
                    className, 
                    methodName);
            }
        }

        generateClass("AutoCommandProcessor", codeBlock.build());
    }

    public void processNARUpdateable(List<Element> updateMethods){
        if(!handleAnnotationUsage(updateMethods, "NARUpdateable")){
            return;
        }
        CodeBlock.Builder codeBlock = CodeBlock.builder()
            .addStatement("$T dashboard = $T.getInstance()", ClassName.get("common.utility.narwhaldashboard", "NarwhalDashboard"), ClassName.get("", "NarwhalDashboard"));
 
        for(Element element : updateMethods){
            String methodName = element.getSimpleName().toString();
            String dataName = element.getAnnotation(NARUpdateable.class).name();
            String className = element.getEnclosingElement().getSimpleName().toString();
            String packageName = processingEnv.getElementUtils().getPackageOf(element.getEnclosingElement()).getQualifiedName().toString();
            boolean hasGetInstance = element.getEnclosingElement().getEnclosedElements().stream().anyMatch(e -> e.getSimpleName().toString().equals("getInstance"));
            
            if (!hasGetInstance) {
                continue;
            }
    

            codeBlock.addStatement("dashboard.addUpdate($S, () -> $T.getInstance().$L())", dataName, ClassName.get(packageName, className), methodName);
        }

        generateClass("NARUpdateableProcessor", codeBlock.build());
    }

    public boolean generateClass(String className, CodeBlock... codeBlocks){
        MethodSpec.Builder method = MethodSpec.methodBuilder("process")
        .addModifiers(Modifier.PUBLIC, Modifier.STATIC)
        .returns(void.class);

        for(CodeBlock codeBlock : codeBlocks){
            method.addCode(codeBlock);
        }

        TypeSpec generatedClass = TypeSpec.classBuilder(className)
            .addModifiers(Modifier.PUBLIC)
            .addMethod(method.build())
            .build();

        JavaFile javaFile = JavaFile.builder("processor", generatedClass).build();
    
        try {
            javaFile.writeTo(processingEnv.getFiler());
            if (!className.equals("ClassManager")) {
                generatedClassList.add("processor." + className);
            }
            return true;
        } catch (Exception e) {
            // e.printStackTrace();
            return false;
        }
    }

    public boolean generateClassManger(){
        print("Creating Class Manager...");
        CodeBlock.Builder codeBlock = CodeBlock.builder();
        for(String className : generatedClassList){
            codeBlock.addStatement("$T.process()", ClassName.get("", className));
        }
        return generateClass("ClassManager", codeBlock.build());
    }

    private void print(String message) {
        processingEnv.getMessager().printMessage(Diagnostic.Kind.OTHER, "Annotation Processor Update: " + message);
    }

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Set.of(
            NARUpdateable.class.getCanonicalName(), 
            ShuffleboardData.class.getCanonicalName(),
            AutoCommand.class.getCanonicalName());
    }
    
}