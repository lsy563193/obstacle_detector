/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "../include/panels/obstacle_recorder_panel.h"

using namespace obstacle_detector;
using namespace std;

ObstacleRecorderPanel::ObstacleRecorderPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("obstacle_recorder") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");

  QString user_name = getenv("USER");
  QString play_icon = QString("/home/") + user_name + QString("/ObstacleDetector/resources/play.png");
  QString stop_icon = QString("/home/") + user_name + QString("/ObstacleDetector/resources/stop.png");

  start_button_ = new QPushButton;
  start_button_->setMinimumSize(50, 50);
  start_button_->setMaximumSize(50, 50);
  start_button_->setIcon(QIcon(play_icon));
  start_button_->setIconSize(QSize(25, 25));
  start_button_->setCheckable(true);

  stop_button_ = new QPushButton;
  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setIcon(QIcon(stop_icon));
  stop_button_->setIconSize(QSize(25, 25));
  stop_button_->setCheckable(true);

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(line);
  layout->addLayout(buttons_layout);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));

  evaluateParams();
}

void ObstacleRecorderPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ObstacleRecorderPanel::start() {
  p_recording_ = true;
  processInputs();
}

void ObstacleRecorderPanel::stop() {
  p_recording_ = false;
  processInputs();
}

void ObstacleRecorderPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
}

void ObstacleRecorderPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("recording", p_recording_);
}

void ObstacleRecorderPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("recording", p_recording_, false);
}

void ObstacleRecorderPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  start_button_->setEnabled(p_active_ && !p_recording_);
  start_button_->setChecked(p_recording_);

  stop_button_->setEnabled(p_active_ && p_recording_);
  stop_button_->setChecked(!p_recording_);
}

void ObstacleRecorderPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ObstacleRecorderPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstacleRecorderPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ObstacleRecorderPanel, rviz::Panel)
