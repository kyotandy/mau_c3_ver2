#include "fuzzy_controller.h"
#include "AlgebraicProduct.h"  // AlgebraicProduct のヘッダー
#include "Minimum.h"  // Minimum のヘッダー
#include "Centroid.h"



FuzzyController::FuzzyController() {
    engine = new fl::Engine;
    engine->setName("MotorControl");

    fl::InputVariable* offset = new fl::InputVariable;
    offset->setName("offset");
    offset->setRange(-50.0, 50.0);
    offset->addTerm(new fl::Trapezoid("negative", -50.0, -50.0, -5.0, 0.0));
    offset->addTerm(new fl::Trapezoid("zero", -3.0, -1.0, 1.0, 3.0));
    offset->addTerm(new fl::Trapezoid("positive", 0.0, 5.0, 50.0, 50.0));
    engine->addInputVariable(offset);

    fl::InputVariable* angle = new fl::InputVariable;
    angle->setName("angle");
    angle->setRange(-45.0, 45.0);
    angle->addTerm(new fl::Trapezoid("negative", -45.0, -45.0, -10.0, 0.0));
    angle->addTerm(new fl::Trapezoid("zero", -10.0, -2.0, 2.0, 10.0));
    angle->addTerm(new fl::Trapezoid("positive", 0.0, 10.0, 45.0, 45.0));
    engine->addInputVariable(angle);

    fl::OutputVariable* motor_speed = new fl::OutputVariable;
    motor_speed->setName("motor_speed");
    motor_speed->setRange(-100.0, 100.0);
    motor_speed->addTerm(new fl::Triangle("negative", -100.0, -100.0, 0.0));
    motor_speed->addTerm(new fl::Triangle("zero", -90.0, 0.0, 90.0));
    motor_speed->addTerm(new fl::Triangle("positive", 0.0, 100.0, 100.0));

    motor_speed->setDefuzzifier(new fl::Centroid(100));
    motor_speed->setDefaultValue(fl::nan); // デフォルト値設定
    motor_speed->setAggregation(new fl::Maximum);
    engine->addOutputVariable(motor_speed);

    fl::RuleBlock* ruleBlock = new fl::RuleBlock;
    ruleBlock->addRule(fl::Rule::parse("if offset is negative and angle is negative then motor_speed is positive", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is negative and angle is zero then motor_speed is positive", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is negative and angle is positive then motor_speed is negative", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is zero and angle is negative then motor_speed is positive", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is zero and angle is zero then motor_speed is zero", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is zero and angle is positive then motor_speed is negative", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is positive and angle is negative then motor_speed is positive", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is positive and angle is zero then motor_speed is negative", engine));
    ruleBlock->addRule(fl::Rule::parse("if offset is positive and angle is positive then motor_speed is negative", engine));

    ruleBlock->setConjunction(new fl::AlgebraicProduct);
    ruleBlock->setImplication(new fl::Minimum);
    engine->addRuleBlock(ruleBlock);
}

float FuzzyController::compute(float offset_value, float angle_value) {
    engine->getInputVariable("offset")->setValue(offset_value);
    engine->getInputVariable("angle")->setValue(angle_value);
    engine->process();
    return engine->getOutputVariable("motor_speed")->getValue();
}
