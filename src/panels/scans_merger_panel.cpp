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

#include "../include/panels/scans_merger_panel.h"

using namespace obstacle_detector;
using namespace std;

ScansMergerPanel::ScansMergerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("scans_merger") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  scan_checkbox_ = new QCheckBox("Publish scan");
  pcl_checkbox_ = new QCheckBox("Publish PCL");
  set_button_ = new QPushButton("Set");

  n_input_ = new QLineEdit();
  r_min_input_ = new QLineEdit();
  r_max_input_ = new QLineEdit();
  x_min_input_ = new QLineEdit();
  x_max_input_ = new QLineEdit();
  y_min_input_ = new QLineEdit();
  y_max_input_ = new QLineEdit();
  frame_id_input_ = new QLineEdit();

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* checks_layout = new QHBoxLayout;
  checks_layout->addItem(margin);
  checks_layout->addWidget(scan_checkbox_);
  checks_layout->addItem(margin);
  checks_layout->addWidget(pcl_checkbox_);
  checks_layout->addItem(margin);

  QHBoxLayout* n_layout = new QHBoxLayout;
  n_layout->addItem(margin);
  n_layout->addWidget(new QLabel("Number of ranges:"));
  n_layout->addWidget(n_input_);
  n_layout->addItem(margin);

  QHBoxLayout* r_layout = new QHBoxLayout;
  r_layout->addItem(margin);
  r_layout->addWidget(new QLabel("r<sub>min</sub>:"));
  r_layout->addWidget(r_min_input_);
  r_layout->addWidget(new QLabel("m, "));
  r_layout->addWidget(new QLabel("r<sub>max</sub>:"));
  r_layout->addWidget(r_max_input_);
  r_layout->addWidget(new QLabel("m"));
  r_layout->addItem(margin);

  QHBoxLayout* x_layout = new QHBoxLayout;
  x_layout->addItem(margin);
  x_layout->addWidget(new QLabel("x<sub>min</sub>:"));
  x_layout->addWidget(x_min_input_);
  x_layout->addWidget(new QLabel("m, "));
  x_layout->addWidget(new QLabel("x<sub>max</sub>:"));
  x_layout->addWidget(x_max_input_);
  x_layout->addWidget(new QLabel("m"));
  x_layout->addItem(margin);

  QHBoxLayout* y_layout = new QHBoxLayout;
  y_layout->addItem(margin);
  y_layout->addWidget(new QLabel("y<sub>min</sub>:"));
  y_layout->addWidget(y_min_input_);
  y_layout->addWidget(new QLabel("m, "));
  y_layout->addWidget(new QLabel("y<sub>max</sub>:"));
  y_layout->addWidget(y_max_input_);
  y_layout->addWidget(new QLabel("m"));
  y_layout->addItem(margin);

  QHBoxLayout* frame_id_layout = new QHBoxLayout;
  frame_id_layout->addItem(margin);
  frame_id_layout->addWidget(new QLabel("Frame ID:"));
  frame_id_layout->addWidget(frame_id_input_);
  frame_id_layout->addItem(margin);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(checks_layout);
  layout->addWidget(lines[1]);
  layout->addLayout(n_layout);
  layout->addLayout(r_layout);
  layout->addWidget(lines[2]);
  layout->addLayout(x_layout);
  layout->addLayout(y_layout);
  layout->addWidget(lines[3]);
  layout->addLayout(frame_id_layout);
  layout->addWidget(lines[4]);
  layout->addWidget(set_button_);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(processInputs()));

  evaluateParams();
}

void ScansMergerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ScansMergerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_publish_scan_ = scan_checkbox_->isChecked();
  p_publish_pcl_ = pcl_checkbox_->isChecked();

  try { p_ranges_num_ = boost::lexical_cast<int>(n_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_ranges_num_ = 0; n_input_->setText("0"); }

  try { p_min_scanner_range_ = boost::lexical_cast<double>(r_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_scanner_range_ = 0.0; r_min_input_->setText("0.0"); }

  try { p_max_scanner_range_ = boost::lexical_cast<double>(r_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_scanner_range_ = 0.0; r_max_input_->setText("0.0"); }

  try { p_min_x_range_ = boost::lexical_cast<double>(x_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_x_range_ = 0.0; x_min_input_->setText("0.0"); }

  try { p_max_x_range_ = boost::lexical_cast<double>(x_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_x_range_ = 0.0; x_max_input_->setText("0.0"); }

  try { p_min_y_range_ = boost::lexical_cast<double>(y_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_y_range_ = 0.0; y_min_input_->setText("0.0"); }

  try { p_max_y_range_ = boost::lexical_cast<double>(y_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_y_range_ = 0.0; y_max_input_->setText("0.0"); }

  p_frame_id_ = frame_id_input_->text().toStdString();
}

void ScansMergerPanel::setParams() {
  nh_local_.setParam("ranges_num", p_ranges_num_);

  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("publish_scan", p_publish_scan_);
  nh_local_.setParam("publish_pcl", p_publish_pcl_);

  nh_local_.setParam("min_scanner_range", p_min_scanner_range_);
  nh_local_.setParam("max_scanner_range", p_max_scanner_range_);
  nh_local_.setParam("max_x_range", p_max_x_range_);
  nh_local_.setParam("min_x_range", p_min_x_range_);
  nh_local_.setParam("max_y_range", p_max_y_range_);
  nh_local_.setParam("min_y_range", p_min_y_range_);

  nh_local_.setParam("frame_id", p_frame_id_);
}

void ScansMergerPanel::getParams() {
  nh_local_.param<int>("ranges_num", p_ranges_num_, 1000);

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("publish_scan", p_publish_scan_, true);
  nh_local_.param<bool>("publish_pcl", p_publish_pcl_, true);

  nh_local_.param<double>("min_scanner_range", p_min_scanner_range_, 0.05);
  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 10.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_,  10.0);
  nh_local_.param<double>("min_x_range", p_min_x_range_, -10.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_,  10.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -10.0);

  nh_local_.param<string>("frame_id", p_frame_id_, "scanner_base");
}

void ScansMergerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  scan_checkbox_->setEnabled(p_active_);
  pcl_checkbox_->setEnabled(p_active_);

  scan_checkbox_->setChecked(p_publish_scan_);
  pcl_checkbox_->setChecked(p_publish_pcl_);

  n_input_->setEnabled(p_active_);
  r_min_input_->setEnabled(p_active_);
  r_max_input_->setEnabled(p_active_);
  x_min_input_->setEnabled(p_active_);
  x_max_input_->setEnabled(p_active_);
  y_min_input_->setEnabled(p_active_);
  y_max_input_->setEnabled(p_active_);
  frame_id_input_->setEnabled(p_active_);

  n_input_->setText(QString::number(p_ranges_num_));
  r_min_input_->setText(QString::number(p_min_scanner_range_));
  r_max_input_->setText(QString::number(p_max_scanner_range_));
  x_min_input_->setText(QString::number(p_min_x_range_));
  x_max_input_->setText(QString::number(p_max_x_range_));
  y_min_input_->setText(QString::number(p_min_y_range_));
  y_max_input_->setText(QString::number(p_max_y_range_));
  frame_id_input_->setText(QString::fromStdString(p_frame_id_));

  set_button_->setEnabled(p_active_);
}

void ScansMergerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ScansMergerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ScansMergerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ScansMergerPanel, rviz::Panel)
