/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

//import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import CameraViewer from "./CameraViewer"
import {createShortValuesFromNamespaces} from "./Utilities"

@inject("ros")
@observer

// MultiImageViewer Application page
class ImageViewerApp extends Component {

  constructor(props) {
    super(props)

    this.createImageTopicsOptions = this.createImageTopicsOptions.bind(this)
    this.onChangeInputImgSelection = this.onChangeInputImgSelection.bind(this)

    this.state = {
      selectedImageTopics: [null,null,null,null],
      selectedImageLabels: [null,null,null,null]
    }
  }

  // Function for creating image topic options.
  createImageTopicsOptions() {
    var items = []
    items.push(<Option>{"None"}</Option>) 
    const { imageTopics } = this.props.ros
    var imageTopicShortnames = createShortValuesFromNamespaces(imageTopics)
    for (var i = 0; i < imageTopics.length; i++) {
      items.push(<Option value={imageTopics[i]}>{imageTopicShortnames[i]}</Option>)
    }
    return items
  }

  onChangeInputImgSelection(event) {
    const idx = event.nativeEvent.target.selectedIndex
    const text = event.nativeEvent.target[idx].text
    const value = event.target.value
    
    var selector_idx = 0
    if (event.nativeEvent.target.id === "ImageSelector_1") {
      selector_idx = 1
    }
    else if (event.nativeEvent.target.id === "ImageSelector_2") {
      selector_idx = 2
    }
    else if (event.nativeEvent.target.id === "ImageSelector_3") {
      selector_idx = 3
    }

    var newSelectedImageTopics = [...this.state.selectedImageTopics]
    newSelectedImageTopics[selector_idx] = (value === "None")? null : value

    var newSelectedImageLabels = [...this.state.selectedImageLabels]
    newSelectedImageLabels[selector_idx] = (text === "None")? null : text

    this.setState({
      selectedImageTopics: newSelectedImageTopics,
      selectedImageLabels: newSelectedImageLabels
    })
  }


  render() {
    const {selectedImageTopics, selectedImageLabels} = this.state

    const imageOptions = this.createImageTopicsOptions()
    const colCount = ((selectedImageTopics[1] !== null) || (selectedImageTopics[2] !== null) || (selectedImageTopics[3] !== null))? 3 : 2
    const selectionFlexSize = (colCount === 3)? 0.6 : 0.3
    
    
    return (
      <Columns>
        <Column>
          <CameraViewer
            imageTopic={selectedImageTopics[0]}
            title={selectedImageLabels[0]}
            hideQualitySelector={false}
          />
          {(selectedImageTopics[2] !== null)?
            <CameraViewer
            imageTopic={selectedImageTopics[2]}
            title={selectedImageLabels[2]}
            hideQualitySelector={false}
          />          
          : null
          }
        </Column>
        {(colCount === 3)?
        <Column>
          <CameraViewer
            imageTopic={selectedImageTopics[1]}
            title={selectedImageLabels[1]}
            hideQualitySelector={false}
          />
          {(selectedImageTopics[3] !== null)?
            <CameraViewer
              imageTopic={selectedImageTopics[3]}
              title={selectedImageLabels[3]}
              hideQualitySelector={false}
            />          
          : null
          }
        </Column>
        : null
        }
        <Column style={{flex: selectionFlexSize}}>
          <Label title={"Img 1"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_0">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 2"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_1">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 3"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_2">
              {imageOptions}
            </Select>
          </Label>
          <Label title={"Img 4"}>
            <Select onChange={this.onChangeInputImgSelection} id="ImageSelector_3">
              {imageOptions}
            </Select>
          </Label>
          <div align={"left"} textAlign={"left"}  >
            <ButtonMenu>
              <Button onClick={this.onEventTriggered}>{"Event Trigger"}</Button>
            </ButtonMenu>
          </div>
        </Column>
      </Columns>
    )
  }
}

export default ImageViewerApp
